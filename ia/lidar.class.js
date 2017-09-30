/**
 * Module de component lidar de l'IA
 *
 * @module ia/lidar
 * @requires log4js
 */

"use strict";

const log4js = require('log4js');
const logger = log4js.getLogger('ia.lidar');
const EventEmitter = require('events');

const SILENCE_TIMEOUT = 1000;					// ms
const ROCKET_SECURITY_DIAM = 150;				// mm

/**
 * Lidar mng de l'IA
 *
 * @class Lidar
 * @memberof module:ia/lidar
 */
class Lidar {

	constructor(ia, params) {
		class Events extends EventEmitter{}
		this.events = new Events();

		/** IA */
		this.ia = ia;
		// /** Our color */
		// this.color = params.color;
		/** We have */
		this.we_have_hats = params.we_have_hats;
		/** Number of hokuyo */
		this.nb_hokuyo = 0;
		/** Status of Lidar */
		this.lidar_status = "";
		/** Dernier timestamp où on a update (changé à la fin de l'update) */
		this.last_lidar_data = 0;
		/** */
		this.status_timer = null;

		this.emergencyStopped_silence = false;
		this.emergencyStopped_status = false;

		/** nb d'itération depuis laquelle on a perdu un robot */
		this.nb_lost = 0;
	}

	updateStatus() {
		clearTimeout(this.status_timer);

		let deltaT = this.ia.timer.get() - this.last_lidar_data;
		if (!this.emergencyStopped_silence
			&& !this.emergencyStopped_status
			&& deltaT > SILENCE_TIMEOUT) {
			this.mayday("Haven't heard Lidar since " + deltaT + "ms");
			this.emergencyStopped_silence = true;
			// Caution, will spam every 200ms !
		}

		if (this.emergencyStopped_silence && deltaT < SILENCE_TIMEOUT) {
			this.resume("Lidar node talks again");
			this.emergencyStopped_silence = false;
		}

		this.status_timer = setTimeout( function(){
			this.updateStatus();
		}.bind(this), 200);
	}

	/**
	 * Starts the hokuyo
	 */
	start() {
		// logger.info("Lidar AI component started, waiting for LiDAR node data...");
		// this.ia.client.send("hokuyo", "start", {color:this.params.color}); // must have been already done

		// logger.debug("TODO: hokuyo, keep track of living hokuyos according to data coming from LiDAR");
		// timeout = setTimeout(function() {this.timedOut(); }.bind(this) , LOST_TIMEOUT*1000);

		// Status loop
		this.updateStatus();
	};

	/**
	 * Stops the Hokuyo
	 */
	stop() {
		this.ia.client.send("hokuyo", "stop", {});
		clearTimeout(this.status_timer);
	};

	mayday(reason){
		this.emergencyStopped_status = true;
		logger.debug("Mayday called, the given reason is " + reason);
		this.events.emit("emergencyStop", reason);
	}


	resume(reason){
		logger.debug("Resume called, the given reason is :" + reason);
		this.events.emit("endOfEmergencyStop", reason);
	}

	getDistance (spot1, spot2) {
		return Math.sqrt(Math.pow(spot1.x - spot2.x, 2) + Math.pow(spot1.y - spot2.y, 2));
	}

	deleteOurRobots(spots){
		for(let i = spots.length - 1; i>= 0; i--) { // TODO: test this loop works worrectly
			if(this.getDistance( { x: spots[i][0], y:spots[i][1] }, this.ia.pr.pos) < this.ia.pr.size.d/2 ||
				this.getDistance( { x: spots[i][0], y:spots[i][1] }, this.ia.gr.pos) < this.ia.gr.size.d/2) {
				// logger.debug("Found one of our robots at [" + spots[i][0] + ", " + spots[i][1] + "]");
				spots.splice(i, 1);
			}
		}
		return spots;
	}

	deleteRockets(spots){
		let c = spots.length;

		for(let s = spots.length - 1; s>= 0; s--) {
			for(var r in this.ia.data.rockets) {
				let dist = this.getDistance( spots[s].pos, this.ia.data.rockets[r]);

				if(dist < ROCKET_SECURITY_DIAM) {
					// logger.debug("Found one of our robots at [" + spots[i][0] + ", " + spots[i][1] + "]");
					spots.splice(s, 1);
					break;
				}
			}
		}
		// logger.debug("Found " + (c-spots.length) + " rockets");
		return spots;
	}

	/**
	* on data coming from Lidar node (old name : pr.updatePos())
	*/
	onAllSpot(robots){
		if (!this.ia.timer.match_started) {
			// logger.warn("We are receiving data but match hasn't started yet");
			return;
		}

		// Convert cm to mm
		robots = robots.map((val) => {
			return {
				pos: {
					x: val[0]*10,
					y: val[1]*10,
				},
				d: 320
			}
		});

		robots = this.deleteRockets(robots);

		if (this.we_have_hats) {
			robots = this.deleteOurRobots(robots);
		}

		// Tell the robots about the ennemy pos, and let them react accordingly
		this.ia.pr.detectCollision(robots.concat(this.ia.gr));
		this.ia.gr.detectCollision(robots.concat(this.ia.pr));

		// Tell the robots about the ennemy pos, and let them react accordingly
		this.ia.pr.actions.killObjects(robots);
		this.ia.gr.actions.killObjects(robots);

		// Update data
		this.ia.data.dots = robots.map(function(val) {
			return {
				pos: {
					x: val.pos.x,
					y: val.pos.y,
				},
				d: 320
			}
		});

		if (robots.length > 0) {
			this.ia.data.erobot[0].pos= {
				x: robots[0].pos.x,
				y: robots[0].pos.y
			};

			if (robots.length > 1) {
				this.ia.data.erobot[1].pos= {
					x: robots[1].pos.x,
					y: robots[1].pos.y
				};
			};
		};
		
		// logger.debug("TODO: check update last_lidar_data");
		this.last_lidar_data = this.ia.timer.get();

		// Status loop
		this.updateStatus();
	}

	onLidarStatus(params){
		/** Number of hokuyo */
		this.nb_hokuyo = params.nb;
		/** Status of Lidar */
		this.lidar_status = params.status;

		if (this.ia.timer.match_started) {		
			if (!this.emergencyStopped_silence
				&&
					(this.nb_hokuyo <= 0
					|| (this.lidar_status != "ok"
						&& this.lidar_status != "everythingIsAwesome"))
				)
			{
				var reason = "LiDAR status " + this.lidar_status + " with " + this.nb_hokuyo + " active hokuyo(s) doesn't allow us to continue";
				this.mayday(reason);
				this.emergencyStopped_status = true;
			}

			if (this.emergencyStopped_status
				&& this.nb_hokuyo > 0
				&& (this.lidar_status == "ok"
					|| this.lidar_status == "everythingIsAwesome")) {
				this.resume("Hokuyo(s) alive");
				this.emergencyStopped_status = false;
			}
		}
	}

	/**
	 * Parse Order
	 * 
	 * @param {string} from
	 * @param {string} name
	 * @param {Object} params
	 */
	parseOrder(from, name, params) {
		// logger.warn(name);
		switch (name){
			case "all":
				// We already treat data in light
				// this.onAllSpot(params.robotsSpots);
				// this.updatePos(params.dots);
				break;
			case "light":
				this.onAllSpot(params.robotsSpots);
				break;
			case "status":
				this.onLidarStatus(params);
				// this.updateNumberOfRobots(params.nb);
				break;
			default:
				logger.warn("Message name " + name + " not understood");
		}
	};
}

module.exports = Lidar;