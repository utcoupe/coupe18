/**
 * Extension controlling the rocket launch (it's the funny action)
 *
 * Created by tfuhrman on 22/05/17.
 *
 * @module clients/Extension/sweeper
 * @requires module:clients/Extension/extension
 * @requires module:clients/Extension/Actuators/servo
 */

"use strict";

const Extension = require('./extension');
var servos = require('../actuators/servo');

class Rocket extends Extension {
    constructor(){
        super ("rocket");
        this.servos = null;
    }

    takeOrder (from, name, param) {
        this.logger.info ("Order received: " + name);
        switch (name) {
            case "funny_action":
                this.fifo.newOrder (() => {
                    this.processFifoOrder("launch", param);
                });
                break;
            default:
                this.logger.error("Order " + name + " does not exist!");
        }
    }

    processFifoOrder (name, param) {
        this.logger.info("Executing order " + name);
        switch (name) {
            case "launch":
                this.servos.launchRocket(() => {
                    this.fifo.orderFinished();
                });
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

module.exports = Rocket;