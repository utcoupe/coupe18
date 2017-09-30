// TODO :
// 		  message/objet erreur ou pas ?

(function (){
	"use strict";

	/* JS which will connect to the server, and then
	*  will execute the C program to control the Hokuyos.
	*  It will transfer datas from the C Hokuyo controller to AI
	*/

	var log4js = require('log4js');
	var logger = log4js.getLogger('Client');
	var spawn = require('child_process').spawn;
	var fs = require('fs');
	var match_name  = "";
	var child_process = require('child_process');
	var child;
	var SocketClient = require('../server/socket_client.class.js');
	var config = require('../config.js');
	var lastT = Date.now();
	var startingT = lastT;

	var FREQ_ENVOI_INFO = 50; // tous les 10 infos (genre 1 seconde)
	var nth = 0;

	var server = process.argv[2] || config.server;
	var command = process.argv[3] || config.hokuyo_command;

	var client = new SocketClient({
		server_ip: server,
		type: "hokuyo",
	});

	var nb_active_hokuyos = -1;
	var lastStatus = {
		"status": "waiting"
	};
	sendChildren(lastStatus);

	logger.info("Starting with pid " + process.pid);

	client.order(function(from, name, params){
		var now = Date.now();
		logger.info("Time since last order : "+(now - lastT));
		if (now - lastT > 500) { // half a second between two orders
			logger.info("Just received an order `" + name + "` from " + from + " with params :");
			logger.info(params);

			lastT = now;
			switch (name){
				case "start":
					if(!!params.color)
						start(params.color);
					else
						logger.error("Missing parameters !");
					break;
				case "shutdown":
					quitC("stop");
					spawn('sudo', ['halt']);
					break;
				case "kill":
					quitC("stop");
					break;
				case "sync_git":
					spawn('/root/sync_git.sh', [], {
						detached: true
					});
					break;
				default:
					logger.warn("Name not understood : " + data);
			}
		} else {
			logger.warn("Received two orders too closely !");
		}
	});

	function matchLogger(name, line){
		fs.appendFile('/var/log/utcoupe/'+name+'.log', line+'\n', function (err) {
			if (err) logger.error('Ecriture dans le fichier de log de match impossible');
			// logger.debug('The "data to append" was appended to file!');
		});
	}

	function uException(code){
		logger.error("uException sent with code "+code);
	}

	function start(color){
		// We just an order to start, with the flavour :P (color, number of robots)

		sendChildren({"status": "starting"});

		// Generates the match name (for the log file)
		var tmp = new Date();
		match_name = tmp.toJSON().replace(/T/, ' ').replace(/\..+/, '');
		var now = Date.now() - lastT;
		var x1 = 1500, y1 = 400;
		var x2 = 1500, y2 = 600;
		var x1_inc= -20, y1_inc = 50;
		var x2_inc= -25, y2_inc = 35;
		matchLogger(match_name, now+"; color:"+color);
		now = lastT;

		// Functions
		function parseRobots(string) {
			var dots = [];
			now = Date.now() - lastT;

			x1 += x1_inc;
			y1 += y1_inc;

			if (x1 < 1000 || x1 > 1800)
				x1_inc = -x1_inc;
			if (y1 < 200 || y1 > 800)
				y1_inc = -y1_inc;

			x2 += x2_inc;
			y2 += y2_inc;

			if (x2 < 900 || x2 > 1800)
				x2_inc = -x2_inc;
			if (y2 < 600 || y2 > 1200)
				y2_inc = -y2_inc;

			//logger.debug("x:"+x1+" y:"+y1+"  x:"+x1+" y:"+y1);

			dots.push({x: x1, y: y1});
			dots.push({x: x2, y: y2});
			now = lastT;

			// Send all robots
			client.send("ia", "hokuyo.position_tous_robots", {dots: dots});
		}

		function parseInfo(string) {
			logger.info("Read info...");
			// logger.info(string);

			var prev_n_a_h = nb_active_hokuyos;

			nb_active_hokuyos = 2;

			now = Date.now() - lastT;

			if ((prev_n_a_h != nb_active_hokuyos) || (nth == FREQ_ENVOI_INFO)){
				logger.info("Info sent to server");
				sendChildren(getStatus());
				nth = 0;
			}

			matchLogger(match_name, now+"; nb_hokuyo:"+nb_active_hokuyos);
			now = lastT;
			nth += 1;
		}

		function dataFromCHandler(input) {
			parseRobots();
			setTimeout(dataFromCHandler, 100); // call every 100ms
		}

		sendChildren({"status": "starting"});
		dataFromCHandler();
	}

	function getStatus(){
		var data = {
			"status": "",
			"children": []
		};
		
		switch (nb_active_hokuyos){
			case 0:
				data.status = "error";
				break;
			case 1:
				data.status = "ok";
				data.children =  ["Lonesome hokuyo"];
				break;
			case 2:
				data.status = "everythingIsAwesome";
				data.children =  ["Hokuyo 1", "Hokuyo 2"];
				break;
		}

		return data;
	}


	// Sends status to server
	function sendChildren(status){
		lastStatus = status;

		client.send("server", "server.childrenUpdate", lastStatus);
		client.send("ia", "hokuyo.nb_hokuyo", { nb: nb_active_hokuyos });
	}

	function isOk(){
		if(lastStatus.status != "waiting")
			lastStatus = getStatus();
		
		client.send("ia", "isOkAnswer", lastStatus);
		client.send("server", "server.childrenUpdate", lastStatus);
		client.send("ia", "hokuyo.nb_hokuyo", { nb: nb_active_hokuyos });
	}
})();
