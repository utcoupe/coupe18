/**
 * Module permettant de construire la base lunaire
 * 
 * @module clients/Extension/baseconstructor
 * @requires module:clients/Extension/extension
 */

"use strict";

const Extension = require('./extension');

/**
 * Extension permettant de construire la base lunaire
 * 
 * @class BaseConstructor
 * @memberof module:clients/Extension/baseconstructor
 * @extends clients/Extension/extension.Extension
 */
class BaseConstructor extends Extension {
    constructor(){
        super("base_constructor");
        this.servos = null;
        this.ax12 = null;
        this.hasAPreparedModule = false;
        this.pushTowards = "dont";
        this.color = "null";
        this.nbModulesToDrop = 0;
    }

    takeOrder (from, name, param) {
        this.logger.info("Order received : " + name);

        if (!this.started) {
            this.logger.error("BaseConstructor isn't started");
            return;
        }

        switch (name) {
            
            case "prepare_module":
                this.prepareModule(param);
                break;
            
            case "drop_module":
                this.dropModule(param);
                break;
            
            case "clean":
                this.logger.debug("Cleaning FiFo");
                this.fifo.clean();
                break;
            
            case "stop":
                this.stop();
                break;

            case "send_message":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name, param);
                }, name);
                break;
            
            // ************ tests only ! ************
            case "drop":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name);
                }, name);
                break;
            case "engage":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name);
                }, name);
                break;
            case "push":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name, {towards: param.push_towards});
                }, name);
                break;
            case "rotate":
                this.fifo.newOrder(() => {
                    this.processFifoOrder(name, {color: param.color});
                }, name);
                break;
            
            default :
                this.logger.error("Order " + name + " does not exist !");
        }
    }

    processFifoOrder (name, param) {
        this.logger.info("Order executing : " + name);

        if (!this.started) {
            this.logger.error("BaseConstructor isn't started");
            return;
        }

        switch (name) {
            case "drop":
                if (this.nbModulesToDrop > 0) {
                    this.nbModulesToDrop--;
                } else {
                    this.logger.error("Aucun module à déposer !");
                    // this.fifo.orderFinished();
                }
                this.servos.moduleDrop( () => {
                    this.fifo.orderFinished();
                });
                this.fifo.newOrder(() => {
                    this.sendDataToIA("pr.module--", {});
                    this.fifo.orderFinished();
                }, "sendModule--");
                break;
            case "engage":
                this.servos.moduleEngage( () => {
                    this.fifo.orderFinished();
                });
                break;
            case "rotate":
                this.servos.moduleRotate( () => {
                    this.fifo.orderFinished();
                }, param);
                break;
            case "push":
                /// TODO AX12 action with param.towards
                this.logger.error("TODO: do AX12 push action");
                switch(param.towards){
                    case "dont":
                        this.ax12.sendDummyCenter(() => {
                            this.fifo.orderFinished();
                        });
                        break;
                    case "left":
                        this.ax12.sendDummyLeft(() => {
                            this.fifo.orderFinished();
                        });
                        break;
                    case "right":
                        this.ax12.sendDummyRight(() => {
                            this.fifo.orderFinished();
                        });
                        break;
                }
                this.fifo.orderFinished();
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
            this.logger.error("Servos must be provided to BaseConstructor");
        }
        if (!!actuators.ax12) {
            this.ax12 = actuators.ax12;
        } else {
            this.logger.error("AX12 must be provided to BaseConstructor");
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

    /**
     * Engage a module in drop servos and rotate it
     * 
     * @param {Object} [params]
     * @param {String} params.color
     * @param {String} params.push_towards
     */
    prepareModule (params) {
        /*if (params && params.push_towards)
            this.pushTowards = params.push_towards;
        // push_towards -> oposite direction
        var opositPush = "dont";
        if (this.push_towards == "left")
            opositPush = "right";
        else if (this.push_towards == "right")
            opositPush = "left";
        this.fifo.newOrder(() => {
            this.processFifoOrder("push", {towards: opositPush});
        }, "push");*/
        /*this.fifo.newOrder(() => {
            this.processFifoOrder("push", {towards: "right"});
        }, "push");
        // open
        this.fifo.newOrder(() => {
            this.processFifoOrder("engage");
        }, "engage");
        // rotate
        if (params && params.color)
            this.color = params.color;
        if (this.color != "null")
            this.fifo.newOrder(() => {
                this.processFifoOrder("rotate", {color: this.color})
            });
        this.hasAPreparedModule = true;
        */

        // Prepare to take module
        this.fifo.newOrder(() => {
            this.processFifoOrder("engage");
        }, "engage");

        // Move arm
        this.fifo.newOrder(() => {
            this.processFifoOrder("push", {towards: "right"});
        }, "push");

        // Engage module in front of light
        this.fifo.newOrder(() => {
            this.processFifoOrder("drop");
        }, "drop");
        
        // Change to easier position to rotate
        this.fifo.newOrder(() => {
            this.processFifoOrder("engage");
        }, "engage");

        // Find color
        if (params && params.color)
            this.color = params.color;
        if (this.color != "null")
            this.fifo.newOrder(() => {
                this.processFifoOrder("rotate", {color: this.color})
            }, "rotate");

    }

    /**
     * Drop a module after preparing it (according to prepare_module)
     * 
     * @param {Object} params 
     * @param {Number} params.nb_modules_to_drop number of iterations
     */
    dropModule (params) {
        /*for (var idModule = 0; idModule < params.nb_modules_to_drop; idModule++) {
            this.nbModulesToDrop++;
            // Preparation
            if (!this.hasAPreparedModule)
                this.prepareModule();
            // close
            this.fifo.newOrder(() => {
                this.processFifoOrder("drop");
            }, "drop");
            this.fifo.newOrder(() => {
                this.processFifoOrder("push", {towards: this.push_towards});
            }, "push");
            this.hasAPreparedModule = false;
        }*/
        this.client.send("pr", "asserv.pwm", {left : 50, right : 50, ms : 300});
                // Wait the action to be done
                setTimeout(() => {
                    this.fifo.orderFinished();
                },1100);

	setTimeout( () => {
	        this.fifo.newOrder(() => {
	            this.processFifoOrder("drop");
	        }, "drop");
	        this.fifo.newOrder(() => {
	            this.processFifoOrder("engage");
	        }, "engage");
        	this.fifo.newOrder(() => {
	            this.processFifoOrder("push", {towards: "left"});
        	}, "push");
	        this.fifo.newOrder(() => {
        	    this.processFifoOrder("push", {towards: "right"});
	        }, "push");

	}, 300);

    }
}

module.exports = BaseConstructor;
