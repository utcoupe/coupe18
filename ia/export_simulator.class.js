/**
 * Export simulator Module
 * 
 * @module ia/export_simulator
 * @requires log4js
 * @see {@link ia/export_simulator.ExportSimulator}
 */

module.exports = (function () {
	"use strict";
	var log4js = require('log4js');
	var logger = log4js.getLogger('ia.export_simulator');

	var __timeout = null;
	var FPS = 30;
	/** Color of the team */
	var color;

	/**
	 * ExportSimulator Constructor
	 * 
	 * @exports ia/export_simulator.ExportSimulator
	 * @constructor
	 * @param ia
	 */
	function ExportSimulator(ia) {
		/** IA */
		this.ia = ia;
		this.start();
		color = this.ia.color;
	}

	// /**
	//  * Convert x, depends on the team color
	//  * 
	//  * @param {int} x
	//  */
	// function convertX(x) {
	// 	if(color == "yellow") {
	// 		return (x-1500)/1000;
	// 	} else {
	// 		return (1500-x)/1000;
	// 	}
	// }

	/**
	 * Convert x, depends on the team color
	 * 
	 * @param {int} x
	 */
	function convertX(x) {
		return (x-1500)/1000;
	}

	// /**
	//  * Convert y
	//  * 
	//  * @param {int} y
	//  */
	// function convertY(y) {
	// 	return (1000-y)/1000;
	// }
	/**
	 * Convert y
	 * 
	 * @param {int} y
	 */
	function convertY(y) {
		// return (1000-y)/1000;
		return (y-1000)/1000;
	}

	// /**
	//  * Convert Angle, depends on the team color
	//  * 
	//  * @param {int} a Angle
	//  */
	// function convertA(a) {
	// 	if(color == "yellow") {
	// 		return a;
	// 	} else {
	// 		return (a < 0) ? -Math.PI - a : Math.PI - a;
	// 	}
	// }

	/**
	 * Start
	 */
	ExportSimulator.prototype.start = function() {
		this.orderToSimu();
	}
	/**
	 * Stop
	 */
	ExportSimulator.prototype.stop = function() {
		clearTimeout(__timeout);
	}

	/**
	 * Order to Simulator
	 */
	ExportSimulator.prototype.orderToSimu = function() {
		var data = {};
		
		data.robots = {
			gr: {
				x: convertX(this.ia.gr.pos.x),
				y: convertY(this.ia.gr.pos.y),
				a: this.ia.gr.pos.a, // convertA(this.ia.gr.pos.a)
				path: [this.ia.gr.pos].concat(this.ia.gr.path).map(function(pos){
					return [convertX(pos.x), convertY(pos.y)];
				}),
				color: this.ia.color
			},
			pr: {
				x: convertX(this.ia.pr.pos.x),
				y: convertY(this.ia.pr.pos.y),
				a: this.ia.pr.pos.a, // convertA(this.ia.pr.pos.a),
				path: [this.ia.pr.pos].concat(this.ia.pr.path).map(function(pos){
					return [convertX(pos.x), convertY(pos.y)];
				}),
				color: this.ia.color
			},
			egr: {
				x: convertX(this.ia.data.erobot[0].pos.x),
				y: convertY(this.ia.data.erobot[0].pos.y),
				color: (this.ia.color == "yellow") ? "blue" : "yellow"
			},
			epr: {
				x: convertX(this.ia.data.erobot[1].pos.x),
				y: convertY(this.ia.data.erobot[1].pos.y),
				color: (this.ia.color == "yellow") ? "blue" : "yellow"
			}
		};
		// data.dynamic = this.ia.data.dynamic.map(function(o){
		// 	return [convertX(o.pos.x), convertY(o.pos.y)];
		// });
		// logger.debug(data.robots.egr);

		this.ia.client.send("webclient", "simulateur", data);

		__timeout = setTimeout(function(){this.orderToSimu()}.bind(this), 1000/FPS);
	}

	return ExportSimulator;
})();