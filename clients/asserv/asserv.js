/**
 * Module exportant la classe abstraite Asserve
 *
 * @module clients/Asserv/Asserv
 */

"use strict";

const Log4js = require('log4js');
const Fifo = require('../shared/fifo');

class Asserv{
	constructor(robot) {
        // Robot is used to send data
        this.robot = robot;
        this.robotName = this.robot.getName();
        this.logger = Log4js.getLogger(this.robotName + '_asserv');
		this.pos = {};
        this.fifo = new Fifo();
		// this.orderInProgress = false;

		this.getInitPos();
	}

	stop() {
		this.logger.warn("TODO: stop asserv");
	}

	/**
	 * Convert Angle
	 *
	 * @param {int} a Angle
	 */
	convertA(a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
	}

	/**
	 * Set Angle
	 *
	 * @param {int} a Angle
	 */
	setA(a) {
		this.pos.a = this.convertA(a);
	}

	/**
	 * Sets the position and angle
	 *
	 * @param {Object} pos
	 */
	setPos(pos){}

	/**
	 * Gets initial position
	 *
	 */
	getInitPos() {
        this.robot.sendDataToIA(this.robotName+'.getinitpos');
	}


	/**
	 * Sends Position
	 */
	sendPos() {
        this.robot.sendDataToIA(this.robotName+'.pos', this.pos);
	}

	/**
	 * Calage X
	 *
	 * @param {int} x
	 * @param {int} a Angle
	 */
	calageX(x, a){}

	/**
	 * Calage Y
	 *
	 * @param {int} y
	 * @param {int} a Angle
	 */
	calageY(y, a){}

	/**
	 * Set Vitesse
	 *
	 * @param {int} v Speed
	 * @param {float} r Rotation
	 */
	setSpeed(v, r){}

	/**
	 * Speed ?
	 *
	 * @param {int} l
	 * @param {int} a Angle
	 * @param {int} ms
	 */
	speed(l, a, ms){}

	/**
	 * Pulse Width Modulation
	 *
	 * @param {int} left
	 * @param {int} right
	 * @param {int} ms
	 */
	pwm(left, right, ms){}

	/**
	 * Go X Y
	 *
	 * @param {int} x
	 * @param {int} y
	 * @param {string} direction
	 */
	goxy(x, y, direction){}

	/**
	 * Simu Go Angle
	 *
	 * @param {int} a Angle
	 */
	goa(a, callback){}

	/**
	 * Set P I D
	 *
	 * @param {int} p
	 * @param {int} i
	 * @param {int} d
	 */
	setPid(p, i, d){}

	doStartSequence(params) {}
    
	/**
	 * Stops the robot + clean fifo
	 * 
	 * @param {any} activate
	 */
	setEmergencyStop (activate) {
		if (activate) {
			this.clean();
			this.logger.warn("Emergency stop activated!");
		}
		else
			this.logger.warn("Emergency stop disactivated!");
		this.fifo.orderFinished();
	}

    callCallback(callback) {
        callback();
        this.fifo.orderFinished();
    }

    pause () {}

    resume () {}

	addOrderToFifo(name, params){
        this.logger.debug("Adding order to fifo : " + name);
        // this.logger.debug(order.params);
        var callback;
        switch(name) {
            case "send_message":
                callback = function() {
                    this.robot.sendDataToIA(params.name, params || {});
                    //todo, will not work...
                    this.fifo.orderFinished();
                }.bind(this);
                break;
            case "pwm":
                callback = function() {this.pwm(params.left, params.right, params.ms)}.bind(this);
                break;
            case "setvit":
                callback = function() {this.setSpeed(params.v, params.r)}.bind(this);
                break;
            case "setacc":
                callback = function() {this.setAcc(params.acc)}.bind(this);
                break;
            case "clean":
                this.fifo.clean(this.fifo.orderFinished);
                break;
            case "goa":
                callback = function() {this.goa(params.a)}.bind(this);
                break;
            case "goxy":
                callback = function() {this.goxy(params.x, params.y, params.direction)}.bind(this);
                break;
            case "speed":
                callback = function() {this.speed(params.l, params.a, params.ms)}.bind(this);
                break;
            case "setpos":
                callback = function() {this.setPos(params)}.bind(this);
                break;
            case "setpid":
                callback = function() {this.setPid(params.p, params.i, params.d)}.bind(this);
                break;
            case "collision":
				// When a collision, bypass the regular way to send orders to asserv
                callback = function () {};
                this.setEmergencyStop(params.activate);
				break;
            case "callCallback":
                callback = function() {this.callCallback(params.callback)}.bind(this);
                break;
            default:
                this.logger.fatal("This order is unknown for the " + this.robotName + " asserv : " + name);
        }
        if (callback !== undefined) {
            this.fifo.newOrder(callback, name);
        }
	}
}

module.exports = Asserv;
