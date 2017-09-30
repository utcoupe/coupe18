/**
 * Pathfinding module
 * 
 * @module ia/pathfinding
 * @requires log4js
 * @requires path
 * @requires child_process
 * @requires byline
 * @see {@link ia/pathfinding.Pathfinding}
 */

module.exports = (function () {
	"use strict";

	var Path = require('path');

	// var program = Path.normalize(process.env["UTCOUPE_WORKSPACE"] + "/bin/pathfinding");
	var program = Path.normalize(process.env["UTCOUPE_WORKSPACE"] + "/bin/pathfinding");
	// var image = Path.normalize(process.env["UTCOUPE_WORKSPACE"] + "/pathfinding/img/map-20mm-yellow.bmp");
	var image = Path.normalize(process.env["UTCOUPE_WORKSPACE"] + "/pathfinding/img/map.bmp");
	var RATIO = 20;
	var SEPARATOR = ";";

	var logger = require('log4js').getLogger('ia.pathfinding');
	var Child_process = require('child_process');
	var Byline = require('byline');

	/**
	 * Pathfinding constructor
	 * 
	 * @exports ia/pathfinding.Pathfinding
	 * @constructor
	 * @param {Ia} ia
	 */
	function Pathfinding(ia) {
		/** Ia */
		this.ia = ia;
		var callbacksFifo = [];

		this.busy = false;			// true if a request to the C program has already been made and hasn't returned yet
		this.pendingQueries = [];	// fifo queue of queries to the pathfinding


		/*var instance_pkill = Child_process.spawn("pkill", ["pathfinding"]);
		instance_pkill.on('error', function(err){
			logger.error("error pkilling pathfinding code:"+err.code);
		});
		instance_pkill.on('exit', function(code){
			logger.info("successfully pkilled all old instances of pathfinding");
		});


		var instance = Child_process.spawn(program, [ image ]);*/
		var instance = Child_process.spawn("bash", ["-c", "pkill pathfinding;"+program+" -m "+image + " -r"]); // +" -r" for bmp export

		instance.on('error', function(err) {
			if(err.code === 'ENOENT'){
				logger.fatal("pathfinding program executable not found! Is it compiled ? :) Was looking in \""+Path.resolve(program)+"\"");
				process.exit();
			}
			logger.error("c++ subprocess terminated with error:", err);
			console.log(err);
		});
		instance.on('exit', function(code) {
			logger.fatal("c++ subprocess terminated with code:"+code);
		});



		process.on('exit', function(){ //ensure child process is killed
			if(instance.connected){ //and was still connected (dont kill another process)
				instance.kill();
			}
		});

		var stdout = Byline.createStream(instance.stdout);
		stdout.setEncoding('utf8')
		stdout.on('data', function(data) {
			logger.debug("Pathfinding just gave : "+data);
			parse(data);
		});

		instance.stderr.on('data', function(data) {
			logger.error("stderr :"+data.toString());
		});

		/**
		 * Vector Multiply
		 * 
		 * @param {Object} arr
		 * @param {int} ratio
		 */
		function vecMultiply(arr, ratio){
			return arr.map(function(val){
				return Math.round(val*ratio);
			});
		}

		/**
		 * Send a Query
		 * 
		 * @param start
		 * @param end
		 * @param cb
		 */
		this.sendQuery = function(start, end, cb){
			callbacksFifo.push(cb || true);


			var str = ["C"].concat( vecMultiply(start, 1/RATIO) ).concat( vecMultiply(end, 1/RATIO) ).join(SEPARATOR) + "\n";
			instance.stdin.write( str );
			logger.debug("Query to pathfinding from [" + start[0] + ", " + start[1] + "] to [" + end[0] + ", " + end[1] + "]: " + str);
		};

		/**
		 * Send Dynamic
		 * 
		 * @param {Object} objects
		 */
		this.sendDynamic = function(objects){
			//X0, Y0, R0, ...
			var str = ["D"].concat(objects.reduce(function(acc, obj){
				return acc.concat( vecMultiply(obj, 1/RATIO) );
			}, [])).join(SEPARATOR) + "\n";
			// logger.debug(str);
			instance.stdin.write(str);
		}

		/**
		 * Parse data
		 * 
		 * @param {Object} data
		 */
		function parse (data) {
			// X0; Y0; ... Xn; Yn
			var ret = null;
			if(data != "FAIL") {
				var path = [];
				var splitted = data.split(SEPARATOR);
				while(splitted.length > 1){
					path.push( vecMultiply([splitted.shift(), splitted.shift()], RATIO) );
				}

				
				if(path.length > 0) ret = path;
			}

			var callback = callbacksFifo.shift();
			callback(ret); // if(typeof callback === "function") 
		}

	}

	/**
	 * Get Path
	 * 
	 * @param start
	 * @param end
	 * @param callback
	 */
	Pathfinding.prototype.getPath = function (start, end, robot, object, callback) {
		var queryParams = {
			start: start,
			end: end,
			robot: robot,
			object : object,
			callback: callback
		};

		if (this.busy) {
			if (this.pendingQueries.length > 2) {
				logger.warn("Pathfinding queue is getting long (more than 2 requests waiting)");
			}
			// logger.debug("Queued !");
			this.pendingQueries.push(queryParams);
			// logger.debug(this.pendingQueries);
		} else {
			this.prepareAndDoQuery(queryParams);
		}
	};

	/**
	 * prepareAndDoQuery
	 * DO NOT call this from outside, call getPath instead
	 * 
	 * @param start
	 * @param end
	 * @param callback
	 */
	Pathfinding.prototype.prepareAndDoQuery = function(params) {
		this.busy = true;

		this.ia.pathfinding.updateMap(params.robot, params.object);

		// Leave 1000 ms to the C program to answer, then abort
		this.timeout_getpath = setTimeout(function() {
			logger.warn("Pathfinding failed to answer in 1s !");
			this.busy = false;
			params.callback(null);
			params.callback = function() {}; // utile ?

			let nextQuery = this.pendingQueries.shift();
			if (!!nextQuery) {
				this.prepareAndDoQuery(nextQuery);
			}
		}.bind(this), 1000);
		this.sendQuery([params.start.x, params.start.y], [params.end.x, params.end.y], function(path){
			clearTimeout(this.timeout_getpath);
			this.busy = false;

			if(path !== null) {
				path.shift();
				path = path.map(function(val) {
					return {
						x: val[0],
						y: val[1]
					};
				});
			}
			params.callback(path);

			let nextQuery = this.pendingQueries.shift();
			if (!!nextQuery) {
				this.prepareAndDoQuery(nextQuery);
			}
		}.bind(this));
	};

	/**
	 * Borne
	 * 
	 * @param {int} x
	 * @param {int} min
	 * @param {int} max
	 */
	function borne(x, min, max) {
		return x > max ? max : x < min ? min : x;
	}
	
	/**
	 * Update Map
	 */
	Pathfinding.prototype.updateMap = function (robot, except) {
		//[ [x, y, r], ... ]

		// var objects = [];
		// objects.push();
		let otherRobot = (robot.name == this.ia.pr.name) ? this.ia.gr : this.ia.pr;
		var objects = [{
			pos: otherRobot.pos,
			d: otherRobot.size.d
		}].concat(this.ia.data.dots)

		// Add craters
		if (robot.name == this.ia.pr.name) {
			let ballsArray = [];
			for (let name in this.ia.data.balls) {
				ballsArray.push(this.ia.data.balls[name]);
			}
			objects = objects.concat(ballsArray);
		}

		// Add modules still on the table
		let modulesArray = [];
		for (let name in this.ia.data.module) {
			let m = this.ia.data.module[name];
			if ((m.status == "initial") && !((m.pos.x == except.x) && (m.pos.y == except.y))) {
				modulesArray.push(this.ia.data.module[name]);
			}
		}
		objects = objects.concat(modulesArray);
		logger.debug("Exception :" + [except.x, except.y]);
		logger.debug(objects);

		this.sendDynamic( objects.map(function(val){
			// logger.debug(val);
			return [borne(val.pos.x, 0, 2980), borne(val.pos.y, 0, 1980), 0.8*((val.d/2)+(robot.size.d/2))]; // 1*((val.d/2)+(robot.size.d/2))
		}.bind(this)) );
	};

	return Pathfinding;
})();

/*
(function(){
	var pathfinding = new module.exports();
	function log(result){
		console.log("RESULT", result);
		process.exit();
	}

	setTimeout(function(){
		pathfinding.getPath([500,1000], [1500, 200], log);
	}, 10);
})();
//*/
