 /**
 * Module du petit robot de l'IA
 *
 * @module ia/pr
 * @requires module:ia/robot
 * @requires log4js
 * @see {@Link ia/pr.Pr}
 */

"use strict";

const Robot = require('./robot.class.js');

const log4js = require('log4js');
const logger = log4js.getLogger('ia.pr');

/**
 * Petit Robot dans l'IA
 *
 * @class Pr
 * @memberof module:ia/pr
 * @extends {ia/robot.Robot}
 */
class Pr extends Robot{

	constructor(ia, color){
		super(ia, color);

		/** Robot name */
		this.name = "pr";

		/** Size of the robot */
		this.size = {
			l: 170,
			L: 220,
			d: 280
		};

		/** Initial position */
		if (this.ia.color == "blue") {
			this.initialPos = {
				x: 965,
                y: 157,
                a: - Math.PI / 2
			}
			this.startPos = {
				x: 965,
                y: 180,
                a: - Math.PI / 2,
				direction: "backward"
			}
		} else {
			this.initialPos = {
				x: 2035,
				y: 157,
				a: - Math.PI / 2
			}
			this.startPos = {
				x: 2035,
				y: 180,
				a: - Math.PI / 2,
				direction: "backward"
			}
		}

		this.pos.x = this.initialPos.x;
		this.pos.y = this.initialPos.y;
		this.pos.a = this.initialPos.a;

		/** This robot content */
		this.content = {
			nb_modules: 0,
			modules_color: this.ia.color
		};

		/** Robot actions */
		this.actions = new (this.Actions)(this.ia, this);
	}

	// /**
	//  * Place
	//  */
	// place  () {
	// 	// logger.debug('place');
	// 	this.sendInitialPos();
	// 	this.ia.client.send('pr', 'placer');
	// };


	parseOrder (from, name, params) {
		if (super.parseOrder(from, name, params)) { return; }
		var orderNameParts = name.split('.');
		var name = orderNameParts.shift();
		var orderSubname = orderNameParts.join('.');

		switch(name) {
			case 'module++':
				this.content.nb_modules += 1;
				if (this.content.modules_color != "both"
					&& this.actions.inprogress.object.color == "both") {
					// We only have blue or yellow modules
					// but we take a multicolor module
					this.content.modules_color = "both";
				}
			break;
			case 'module--':
				this.content.nb_modules -= 1;
				if (this.content.nb_modules == 0) {
					// If we don't have any module left, we consider having only modules of our color
					// until we take a multicolor module
					this.content.modules_color = this.ia.color;
				}
			break;
			default:
				logger.warn('Unknown order in ia.pr: '+name);
			break;
		}
	};
}

module.exports = Pr;
