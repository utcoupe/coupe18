/**
 * Timer Module
 * 
 * @module ia/timer
 * @requires log4js
 * @see {@link ia/timer.Timer}
 */

module.exports = (function () {
	"use strict";
	const logger = require('log4js').getLogger('ia.timer');

	/**
	 * Timer Constructor
	 * 
	 * @exports ia/timer.Timer
	 * @constructor
	 * @param {Object} ia
	 */
	function Timer(ia) {
		/** t0 */
		this.t0 = null;
		/** Match started */
		this.match_started = false;
		/** IA */
		this.ia = ia;
	}
	/**
	 * Start timer
	 */
	Timer.prototype.start = function() {
		logger.warn("Match starts !");

		this.t0 = Date.now();
		this.match_started = true; // le match commence
		setTimeout(function() {
			logger.fatal("TIME OVER");
			this.match_started = false;
			this.ia.timeout();
			
			setTimeout(function() {
				this.ia.stop();
			}.bind(this), 5000);
		}.bind(this), 89500); 	// the match ends at 1'30 (90 seconds)
	};


	/**
	 * Get Timer. Permet d'obtenir le temps écoulé en ms
	 */
	Timer.prototype.get = function () {
		if (this.match_started)
			return Date.now() - this.t0;
		else
			return 0;
	};
	/**
	 * Status
	 */
	Timer.prototype.status = function () {
		if (this.match_started) {
			return "Match started";
		}
		else {
			return "Wainting for jack";
		}
	};

	return Timer;
})();

