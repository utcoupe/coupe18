/**
 * Module du canon à balles
 * 
 * @module clients/Extension/canon
 * @requires module:clients/Extension/extension
 * @requires module:clients/Extension/Actuators/servo
 */

"use strict";

const Extension = require('./extension');

class Canon extends Extension {
    constructor(){
        super("canon");
        this.servos = null;
    }

    takeOrder(from, name, params) {
        this.logger.info ("Order received: " + name);
        switch (name) {
            case "throw_balls":
                //TODO does not work yet
                this.fifo.newOrder(() => {
                    this.processFifoOrder("turn_on", params);
                });
                this.fifo.newOrder(() => {
                    this.processFifoOrder("open_trunk", params);
                });
                setTimeout(() => {
                    this.fifo.newOrder (() => {
                        this.processFifoOrder("close_trunk", params);
                    });
                    this.fifo.newOrder(() => {
                        this.processFifoOrder("open_trunk", params);
                    });
                }, 1500);
                setTimeout(() => {
                    this.fifo.newOrder (() => {
                        this.processFifoOrder("close_trunk", params);
                    });
                    this.fifo.newOrder(() => {
                        this.processFifoOrder("open_trunk", params);
                    });
                }, 3000);
                setTimeout(() => {
                    this.fifo.newOrder (() => {
                        this.processFifoOrder("close_trunk", params);
                    });
                    this.fifo.newOrder (() => {
                        this.processFifoOrder("turn_off", params);
                    });
                }, 15000);
                break;
            case "open_trunk":
            case "close_trunk":
                this.fifo.newOrder(() => {
                    this.processFifoOrder (name, params);
                });
                break;
            case "turn_off":
            case "turn_on":
                this.fifo.newOrder(() => {
                    this.processFifoOrder (name, params);
                });
                break;
            case "send_message":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name, params);
                }, name);
                break;
            default:
                this.logger.error ("Order " + name + "does not exist!");
        }
    }

    processFifoOrder (name, param) {
        this.logger.info ("Executing order: " + name);
        switch (name) {
            case "turn_on":
                this.servos.turnOnCanon(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "turn_off":
                this.servos.turnOffCanon(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "open_trunk":
                this.servos.openTrunk(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "close_trunk":
                this.servos.closeTrunk(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "send_message":
                this.sendDataToIA(param.name, param || {});
                this.fifo.orderFinished();
                break;

            default:
                this.logger.error ("Order " + name + " does not exist!");
                this.fifo.orderFinished();
        }
    }

    start(actuators) {
        super.start();
        if (!!actuators.servos) {
            this.servos = actuators.servos;
        } else {
            this.logger.error("Servos must be provided to Canon");
        }
    }

    // Inherited from client
    stop() {
        if (!!this.servos) {
            this.servos.stop();
        }
        super.stop();
    }
}

module.exports = Canon;
