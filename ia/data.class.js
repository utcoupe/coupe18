/**
 * Data Module
 * 
 * @module ia/data
 * @requires log4js
 * @see {@link ia/data.Data}
 */

module.exports = (function () {
	"use strict";
	var log4js = require('log4js');
	var logger = log4js.getLogger('ia.data');

	/**
	 * Data Constructor
	 * 
	 * @exports ia/data.Data
	 * @constructor
	 * @param {Object} ia
	 * @param {int} nb_erobots Number of robots in a team
	 * @param EGR_d
	 * @param EPR_d 
	 */
	function Data(ia) { /*, nb_erobots, EGR_d, EPR_d*/
		this.ia = ia;

		/** balls */
		this.balls = [];
		/** rocket */
		this.rockets = [{
			x:40,
			y:1350
		},{
			x:1150,
			y:40
		},{
			x:1850,
			y:40
		},{
			x:2960,
			y:1350
		}];
		/** module */
		this.module = [];
		/** erobot */
		this.erobot = [];
		/** mountedModules */
		this.mountedModules = [];
		/** throwBalls */
		this.throwBalls = [];
		// /** nb_erobot */
		// this.nb_erobots = nb_erobots;

		// /** craters */
		// this.craters = [{
		// 	pos:{
		// 		x:650,
		// 		y:540
		// 	},
		// 	d: 240
		// },{
		// 	pos:{
		// 		x:2350,
		// 		y:540
		// 	},
		// 	d: 240
		// },{
		// 	pos:{
		// 		x:0,
		// 		y:2000
		// 	},
		// 	d: 510
		// },{
		// 	pos:{
		// 		x:3000,
		// 		y:2000
		// 	},
		// 	d: 510
		// },{
		// 	pos:{
		// 		x:1070,
		// 		y:1870
		// 	},
		// 	d: 240
		// },{
		// 	pos:{
		// 		x:1930,
		// 		y:1870
		// 	},
		// 	d: 240
		// }];

		/** dots */
		this.dots = [];

		this.importObjects();
		
		this.erobot = [{ // big robot on position 0
				name: "gr",
				pos:{
					x:2800,
					y:200
				},
				speed:{ // in mm/sec
					x:0,
					y:0,
				},
				lastUpdate: 0, // time in ms from the beining of the match
				d: 320,
				status: "lost"
			},{ // small robot on position 1
				name: "pr",
				pos:{
					x:2100,
					y:200
				},
				speed:{
					x:0,
					y:0
				},
				lastUpdate: 0,
				d: 200,
				status: "lost"
			}];

		if (this.ia.color == "yellow") {
			this.erobot[0].pos.x = 3000 - this.erobot[0].pos.x;
			this.erobot[1].pos.x = 3000 - this.erobot[1].pos.x;
		}
	}

	/**
	 * Import the Objects
	 */
	Data.prototype.importObjects = function () {
		var ret = require('./objects.json');

		// this.balle = ret.balle;
		// this.chargeur = ret.chargeur;
		// this.clap = ret.clap;
		// this.plot = ret.plot;
		// this.gobelet = ret.gobelet;
		// this.pile = ret.pile;
		// this.depot = ret.depot;

		this.balls = ret.balls;
		this.module = ret.module;
		this.mountedModules = ret.mountedModules;
		this.rocket = ret.rocket;
		this.throwBalls = ret.throwBalls;
		this.seesaw = ret.seesaw;

		return ret;
	};

	/**
	 * Return Object Reference
	 * 
	 * @param {string} name
	 */
	Data.prototype.getObjectRef = function(name) {
		// Retourne une référence vers l'objet name
		// 		name étant de la forme <type>__<nom>
		// Permet d'avoir une référence vers l'objet dans une action

		var actName = name.split("__");
		if (actName.length != 2){
			logger.warn("Le nom '"+name+"' n'est pas un nom d'objet correct.");
			return null;
		}

		if (!this[actName[0]] || !this[actName[0]][actName[1]]){
			logger.warn("L'objet "+actName[1]+" de type "+actName[0]+" est inconnu.");
			logger.warn(this[actName[0]]);
			return null;
		}

		return this[actName[0]][actName[1]];
	};

	/**
	 * Returns true if dist1 is smaller than dist2 
	 * 
	 * @param {int} dist1
	 * @param {int} dist2
	 */
	Data.prototype.isCloser = function (dist1, dist2){
		return (dist1 < dist2);
	};
	
	/**
	 * Parse Order
	 * 
	 * @param {string} from
	 * @param {string} name
	 * @param {Object} param
	 */
	Data.prototype.parseOrder = function(from, name, param){
		switch(name){
			case "add_dynamic" :
				if(!!param.pos && !!param.pos.x && !!param.pos.y && !!param.d){
					this.dynamic.push(param);
					logger.info("added dynamic from :"+from+" params:"+JSON.stringify(param));
				}else{
					logger.error("invalid dynamic from :"+from+" params:"+JSON.stringify(param));
				}				
			break;
			default: logger.error("Unknown order name:"+name+" from:"+from);
		}
	};
	
	return Data;
})();
