/**
 * Module de component robot générique de l'IA
 *
 * @module ia/robot
 */

"use strict";

const log4js = require('log4js');
const logger = log4js.getLogger('ia.robot');

/**
 * Robot générique de l'IA
 *
 * @class Robot
 * @memberof module:ia/robot
 */
class Robot{
	constructor(ia, color){
		/** IA */
		this.ia = ia;

		/** Robot name (pr or gr) */
		this.name = "";

		/** Position */
		this.pos = { // if we are yellow, default, left side of the table
			x: 0,
			y: 0,
			a: 0
		};

		/** Initial position */
		this.initialPos = {
			x: 0,
			y: 0,
			a: 0,
		}

		/** Size of the robot, to be initialized */
		this.size = {
			l: 0,
			L: 0,
			d: 0
		};

		/** Current action */
		this.current_action = null;

		/** Path */
		this.path = [];

		/** Content, to be initialized */
		this.content = {};

		/** We have hats on the top */
		this.we_have_hats = false;
		
		/** Team color */
		this.color = color;

		/** Pause activated and asked to robot */
		this.paused = false;

		/** Robot actions */
		this.actions = null;
		this.Actions = require('./actions.class.js');

		this.startPos = this.initialPos;
	}

	/**
	 * Send initial position
	 */
	sendInitialPos () {
		logger.debug("TODO TEST: check this function");
		this.ia.client.send(this.name, "asserv.setpos", {
			x: this.initialPos.x,
			y: this.initialPos.y,
			a: this.initialPos.a,
			color: this.color
		});
	};

	getDistance (spot1, spot2) {
		return Math.sqrt(Math.pow(spot1.x - spot2.x, 2) + Math.pow(spot1.y - spot2.y, 2));
	};

	detectCollision(dots){
		let segmentsCollision = []; // keeps the collision on every segment of the path
		let currentSegmentIdx = -Infinity; // index of the segment we are currently the closest to
		let shortestDistToRobot = Infinity; // distance to the susmentionned segment
		
		// logger.debug("TODO: detect collision");
		var minDist/*, dotIdx*/;

		var SEGMENT_DELTA_D = 30; // (mm) between 2 iterations on a segment to detect colision
		
		// For each path segment
		var complete_path = [this.pos].concat(this.path);
		for (let i = 0; i < complete_path.length-1; (i++) ) {
			var segment = [complete_path[i], complete_path[i+1]]; // so segment[0][0] is the x value of the beginning of the segment
			var segLength = this.getDistance({x:segment[0].x , y:segment[0].y }, {x:segment[1].x , y:segment[1].y });
			var nthX = (segment[1].x-segment[0].x)*SEGMENT_DELTA_D/segLength; // segment X axis length nth 
			var nthY = (segment[1].y-segment[0].y)*SEGMENT_DELTA_D/segLength;
			
			var distSegmentPtToHokEcho, distSegmentPtToRobot;

			// for each X cm of the segment
			for (var j = 0; (j*SEGMENT_DELTA_D) < segLength; (j++)) {

				var segPoint = {
					x: segment[0].x + nthX*j,
					y: segment[0].y + nthY*j
				};

				// distance to each estimated position of the ennemi robots
				minDist = Infinity;//this.getDistance(dots[0], segPoint);
				
				for(var k = 0; k < dots.length; k++) {
					distSegmentPtToHokEcho = this.getDistance(dots[k].pos, segPoint);
					if (distSegmentPtToHokEcho < minDist) {
						minDist = distSegmentPtToHokEcho;
						//dotIdx = k; // be carefull, we might not see a robot that is ahead on the path but less closer than the other opponent behind us
					}

					distSegmentPtToRobot = this.getDistance(this.pos, segPoint);
					if (distSegmentPtToRobot < shortestDistToRobot) {
						shortestDistToRobot = distSegmentPtToRobot;
						currentSegmentIdx = i;
					}
				}

				// if one of the dist < security diameter, there might be a collision (if this ith segment is ahead of us)
				if (minDist < 450) {
					segmentsCollision.push(true);
				} else {
					segmentsCollision.push(false);
				}
				
			}
		}

		if (currentSegmentIdx < 0) {
			return;
		}

		//logger.debug("Robot is at idx " + currentSegmentIdx + "/" + segmentsCollision.length + ". 1st collision at " + segmentsCollision.indexOf(true));

		// For each path segment THAT IS AHEAD !
		for (let i = currentSegmentIdx; i < segmentsCollision.length; (i++) ) {
			if (segmentsCollision[i]) {
				this.onCollision(/*dots[dotIdx]*/);
				break;
			}
		}
	}

	/**
	 * What to do if a collision happens
	 */
	onCollision() {
		logger.warn('Collision detected between an enemy and ' + this.name /* + ' at [' + pos[0] + ", " + pos[1] + ']'*/ );
		this.path = [];
		this.ia.client.send(this.name, "asserv.collision", {activate: true});

		this.actions.collision();
		this.loop();
	}


	/**
	 * Send initial position
	 */
	sendInitialPos () {
		this.ia.client.send(this.name, "asserv.setpos", this.initialPos);
	};

	/**
	 * Place robot before starting the match
	 */
	place () {
		this.sendInitialPos ()

		this.ia.client.send(this.name, "do_start_sequence", this.startPos);
	}

	/**
	 * Start
	 */
	start () {
		// this.ia.client.send(this.name, "ouvrir_ax12");
		this.loop();
	}

	/**
	 * Loop
	 */
	loop () {
		// Called every time we have finished an action

		// Get the other robot pos not to touch it
		let otherRobotPos = this.name == "pr" ? this.ia.gr.pos : this.ia.pr.pos;

		logger.debug(this.name + ' doing next action');
		this.actions.doNextAction(function() {
			this.loop();
		}.bind(this), otherRobotPos);
	}

	/**
	 * Pause in case of fatal (temporary) loss
	 */
	pause () {
		// Instead of deleting the current action (like collision), we just pause
		this.paused = true;
		this.ia.client.send(this.name, "pause");
	}

	/**
	 * Resume
	 */
	resume () {
		this.paused = false;
		this.ia.client.send(this.name, "resume");
	}

	/**
	 * Stop
	 */
	stop () {
		this.paused = true;
		// logger.debug("Closing " + this.name);
		this.ia.client.send(this.name, 'stop');
	}


	/**
	 * Update the received position
	 */
	onPos (params) {

		function borne(x, min, max) {
			return x > max ? max : x < min ? min : x;
		}

		// logger.debug("New pos : x=" + params.x + ", y=" + params.y + ", a=" + params.a);

		params.x = borne(params.x, 0, 3000);
		params.y = borne(params.y, 0, 2000);
		this.pos = params;

		if (this.path.length > 0
			&& this.getDistance(this.pos, this.path[0]) < this.size.d/2) {
			this.path.shift();
		}
	};


	/**
	 * On message, try to treat it. If not possible, let the child class deal with it
	 */
	parseOrder (from, name, params) {
		var orderNameParts = name.split('.');
		var name = orderNameParts.shift();
		var orderSubname = orderNameParts.join('.');

		switch(name) {
			case 'collision':
				// Manual collision
				this.collision();
			break;
			case 'pos':
				// Asserv told us our position
				this.onPos(params);
			break;
			case 'getinitpos':
				// Robot si asking its initial position
				this.sendInitialPos();
			break;
			case 'place':
				this.place();
			break;
			case 'actions':
				this.actions.parseOrder(from, orderSubname, params);
			break;
			default:
				return false; // we have treated this message, let the child class deal with it
		}

		return true; // we correctly treated this message
	};
}

module.exports = Robot;