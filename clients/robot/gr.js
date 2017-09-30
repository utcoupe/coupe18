/**
 * Module du grand robot
 *
 * @module clients/Robot/grobot
 * @requires module:clients/Robot/robot
 */

"use strict";

const Robot = require('./robot');
const Canon = require('../extension/canon');
const Sweeper = require('../extension/sweeper');
const Rocket = require('../extension/rocket');

/**
 * Grand Robot
 *
 * @class Grobot
 * @memberof module:clients/Robot/grobot
 * @extends {clients/Robot/robot.Robot}
 */
class Grobot extends Robot{

	constructor(robotName){
	    super(robotName);

		this.canon = new Canon();
		this.sweeper = new Sweeper();
        this.rocket = new Rocket();
  	}

	
	// called by start
	openExtensions () {
        this.servo = this.factory.createObject("servo");

        let actuators = {
        	servos: this.servo
        };
        
		this.sweeper.start(actuators);
		this.canon.start(actuators);
        this.rocket.start(actuators);
	}
	
	// called by stop and exit
	closeExtensions () {
		this.sweeper.stop();
		this.canon.stop();
        this.rocket.stop();
	}

	climbSeesaw(callback) {
        // this.asserv.pwm(125, 125, 500);
        //todo add timeout to wait the seesaw to seesaw
        //TODO : color management
        //TODO : ia ?
  //       var orderName = "climbSeesaw";
  //       this.asserv.addOrderToFifo("goxy", {
		// 	x: 780,
		// 	y: 180,
  //           direction : "backward"
		// });
		// // this.asserv.addOrderToFifo("goa", {
		// // 	a: -7*Math.PI/8
		// // });
		// this.asserv.addOrderToFifo("goxy", {
		// 	x: 800,
		// 	y: 180,
  //           direction : "backward"
		// });
		this.asserv.addOrderToFifo("callCallback", {
        	callback: callback
		});
    }

	posArduinoToIa(x, y, a) {
		let pos = {};
        pos.x = x;
        pos.y = y;
        pos.a = a;
        return pos;
    }

	posIaToArduino(pos) {
		let res = {};
        res.x = pos.x;
        res.y = pos.y;
        res.a = pos.a;
        return res;
    }
    


	// Exiting :
	//do something when app is closing
	//process.on('exit', quit);
	// catches ctrl+c event
	//process.on('SIGINT', quit);
	// //catches uncaught exceptions
	// process.on('uncaughtException', quit);
}

module.exports = Grobot;
