/**
 * Module de la balayeuse
 * 
 * @module clients/Extension/sweeper
 * @requires module:clients/Extension/extension
 * @requires module:clients/Extension/Actuators/servo
 */

"use strict";

const Extension = require('./extension');

class Sweeper extends Extension {
    constructor(){
        super("sweeper");
        this.servos = null;
    }

    takeOrder (from, name, param) {
        this.logger.info ("Order received: " + name);
        switch (name) {
            case "send_message":
            case "swallow_balls":
            case "turn_on":
            case "turn_off":
                this.fifo.newOrder (() => {
                    this.processFifoOrder(name, param);
                }, name);
                break;
            
            default:
                this.logger.error("Order " + name + " does not exist!");
        }
    }

    processFifoOrder (name, param) {
        this.logger.info("Executing order " + name);
        switch (name) {
            case "turn_on":
                this.servos.turnOnSweeper(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "turn_off":
                this.servos.turnOffSweeper(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "swallow_balls":
                // PWM is used to go through the table element because the asserv is currently not enough good
                this.client.send("gr", "asserv.pwm", {left : -75, right : -100, ms : 2000});
		this.client.send("gr", "asserv.pwm", {left : 75, right : 100, ms : 2000});
                this.client.send("gr", "asserv.pwm", {left : -100, right : -75, ms : 2000});
                this.client.send("gr", "asserv.pwm", {left : 100, right : 75, ms : 1500});

                // Wait the action to be done
                setTimeout(() => {
                    this.fifo.orderFinished();
		}, 9000);
                setTimeout(() => {
                    this.servos.turnOffSweeper( function() {});
                }, 30000);
                break;
            case "send_message":
                this.sendDataToIA(param.name, param || {});
                this.fifo.orderFinished();
                break;
            default:
                this.logger.error("Order " + name + " does not exist!");
                this.fifo.orderFinished();
        }
    }

    start(actuators) {
        super.start();
        if (!!actuators.servos) {
            this.servos = actuators.servos;
        } else {
            this.logger.error("Servos must be provided to Sweeper");
        }
    }

    stop () {
        if (!!this.servos) {
            this.servos.stop();
        }
        super.stop();
    }
}

module.exports = Sweeper;
