/**
 * Module permettant de ramacer les modules
 *
 * @module clients/Extension/unitgrabber
 * @requires module:clients/Extension/extension
 */

"use strict";

const Extension = require('./extension');

/**
 * Extension permettant de ramasser les modules lunaires
 *
 * @class UnitGrabber
 * @memberof module:clients/Extension/unitgrabber
 * @extends {clients/Extension/extension.Extension}
 */
class UnitGrabber extends Extension {
    constructor(){
        super("unit_grabber");
        this.servos = null;
        this.ax12 = null;
    }

    takeOrder (from, name, param) {
        this.logger.info("Order received " + name);


        if (!this.started) {
            this.logger.error("BaseConstructor isn't started");
            return;
        }

        switch (name) {
            case "take_module":
                this.takeModule();
                break;

            // **************** tests only ************
            case "startArmRotate":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name);
                }, name);
                break;
            case "stopArmRotate":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name);
                }, name);
                break;
            case "openArm":
            case "closeArm":
            case "closeGrabber":            
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name);
                }, name);
                break;
            case "openGrabber":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name);
                }, name);
                break;

            case "send_message":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name, param);
                }, name);
                break;

            case "stop":
                this.stot();
                break;
            case "clean":
                this.logger.debug("Cleaning Fifo...");
                this.fifo.clean();
                break;

            default:
                this.logger.error("Order " + name + " does not exist !");
        }
    }

   processFifoOrder (name, param) {
        this.logger.info("Executing order : " + name);


        if (!this.started) {
            this.logger.error("BaseConstructor isn't started");
            return;
        }

        switch (name) {
            case "startArmRotate":
                this.servos.moduleArmStartRotate(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "stopArmRotate":
                this.servos.moduleArmStopRotate(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "openArm":
                this.servos.moduleArmOpen(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "closeArm":
                this.servos.moduleArmClose(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "initArm":
                this.servos.moduleArmInit(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "closeGrabber":
                // TODO AX12 up
                this.ax12.closeGrabber(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "openGrabber":
                // TODO AX12 down
                this.ax12.openGrabber(() => {
                    this.fifo.orderFinished();
                });
                break;
            case "send_message":
                this.sendDataToIA(param.name, param || {});
                this.fifo.orderFinished();
                break;
            default:
                this.logger.error("Order " + name + " does not exist !");
                this.fifo.orderFinished();
        }
    }

    start(actuators) {
        super.start();
        if (!!actuators.servos) {
            this.servos = actuators.servos;
        } else {
            this.logger.error("Servos must be provided to UnitGrabber");
        }
        if (!!actuators.ax12) {
            this.ax12 = actuators.ax12;
        } else {
            this.logger.error("AX12 must be provided to UnitGrabber");
        }
    }

    // Inherited from client
    stop() {
        if (!!this.servos) {
            this.servos.stop();
        }
        if (!!this.ax12) {
            this.ax12.stop();
        }
        super.stop();
    }

    takeModule () {
        this.fifo.newOrder(() => {
            this.processFifoOrder("startArmRotate");
        }, "startArmRotate");
        this.fifo.newOrder(() => {
            this.processFifoOrder("openArm");
        }, "openArm");
        this.fifo.newOrder(() => {
            this.processFifoOrder("openGrabber");
        }, "openGrabber");
        this.fifo.newOrder(() => {
            this.processFifoOrder("initArm");
        }, "initArm");
        this.fifo.newOrder(() => {
            this.processFifoOrder("openArm");
        }, "openArm");
        this.fifo.newOrder(() => {
            this.processFifoOrder("closeArm");
        }, "closeArm");
        this.fifo.newOrder(() => {
            this.processFifoOrder("openArm");
        }, "openArm");
        this.fifo.newOrder(() => {
            this.processFifoOrder("stopArmRotate");
        }, "stopArmRotate");
        this.fifo.newOrder(() => {
            this.processFifoOrder("closeGrabber");
        }, "closeGrabber");
        this.fifo.newOrder(() => {
            this.processFifoOrder("openGrabber");
        }, "openGrabber");
        this.fifo.newOrder(() => {
            this.processFifoOrder("closeGrabber");
        }, "closeGrabber");
        this.fifo.newOrder(() => {
            this.processFifoOrder("closeArm");
        }, "closeArm");
        this.fifo.newOrder(() => {
            this.sendDataToIA("pr.module++", {});
            this.fifo.orderFinished();
        }, "sendModule++");
    }
}

module.exports = UnitGrabber;
