/**
 *  @file       Describes the ax12 real module.
 *  @date       14/05/2017
 *  @module     clients/actuators/ax12real
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */
"use strict";

/**
 * Mother class, mandatory for inheritance.
 * @type {Ax12}
 */
const Ax12 = require('./ax12');
/**
 * The defineParser module is used to convert C header files #define in an object usable in js.
 */
const defineParser = require("../shared/defineparser");
/**
 * This module is used to spawn C++ process (for ax12).
 */
const Child_process = require('child_process');
/**
 * This module is used to communicate with spawned process, using std/stdout stream.
 */
const Byline = require('byline');

/**
 * Implement the Ax12 class for real robots.
 * The class Ax12Real is a singleton, there is only one instance of it in the system.
 * This class is the interface between the js system (using the public methods of the Ax12 abstract class) and the embedded one.
 * The communication protocol is defined as follow : order;id;arg1;arg2;argn.
 * The orders list is parsed from the header file corresponding to the ax12 program code, using the defineParser object.
 * Before being able to send orders to the ax12 program, this class will check is the ax12 program is launched and responding.
 * The orders received from the ax12 program are handled by the OrdersProcess object.
 * @augments Ax12
 */
class Ax12Real extends Ax12 {
    /**
     * Creates an instance of Ax12.
     */
    constructor() {
        super();
        this.actuatorCommands = defineParser(process.env.UTCOUPE_WORKSPACE + "/ax12/prgm_ax12/src/define.h");
        // this.logger.debug("Launching ax12 cpp");
        const program = process.env.UTCOUPE_WORKSPACE + "/bin/ax12";
        this.ax12 = Child_process.spawn(program);
        this.stdStreamConnected = false;
        this.ax12.on('error', function (err) {
            if (err.code === 'ENOENT') {
                this.logger.fatal("ax12 program executable not found! Is it compiled ? :) Was looking in \"" + program + "\"");
                process.exit();
            }
            this.logger.error("c++ subprocess terminated with error:", err);
            console.log(err);
        }.bind(this));
        this.ax12.on('exit', function (code) {
            this.stdStreamConnected = false;
            this.logger.fatal("c++ subprocess terminated with code:" + code);
        }.bind(this));
        process.on('exit', function () { //ensure child process is killed
            // if(this.ax12.connected){ //and was still connected (dont kill another process)
            if (!!this.ax12) { //and was still connected (dont kill another process)
                this.ax12.kill();
            }
        }.bind(this));
        this.stdout = Byline.createStream(this.ax12.stdout);
        this.stdout.setEncoding('utf8')
        this.stdout.on('data', function (data) {
            this.logger.debug("ax12 just gave : " + data);
            this.parseCommand(data);
        }.bind(this));

        this.ax12.stderr.on('data', function (data) {
            this.logger.error("stderr :" + data.toString());
        }.bind(this));
    }

    /**
     * Function called when a string is received on the inter process port, which means a reception from the ax12 program.
     * At the first reception, ask the ax12 program to start.
     * When an ack of start is received, initialize all the ax12 to know that they are all working fine.
     * The commands received are handled by the OrdersProcess object (this object changes the "data" callback to call,
     * so this function will only be called once when the ax12 program has started).
     * @private
     * @param receivedCommand {string}  Command received, with respect to the ax12 communication protocol
     */
    parseCommand(receivedCommand) {
        if (!this.stdStreamConnected) {
            // If not connected, wait the "ax12" string (telling that the ax12 program is ready) before doing anything else
            if (receivedCommand.indexOf("ax12") == 0) {
                //todo find a way to make it proper
                var order = [this.actuatorCommands.START, 0].join(";") + ";\n";
                this.logger.debug(order);
                this.ax12.stdin.write(order);
            } else {
                this.logger.debug(receivedCommand.toString());
                // Trigger on reception of ack from ax12 program that it has started
                if (receivedCommand.indexOf("0;") == 0) {
                    this.stdStreamConnected = true;
                    // Use the OrderProcess class with stdin and stdout streams
                    this.ordersSerial = require("../shared/orders.process")({
                        in: this.ax12.stdin,
                        out: this.stdout
                    });
                }
                this.logger.debug("INIT movements");
                this.ordersSerial.sendOrder(this.actuatorCommands.AX12_ONE, [this.actuatorCommands.PR_MODULE_GRABBER], () => {
                    this.ordersSerial.sendOrder(this.actuatorCommands.AX12_INIT, [this.actuatorCommands.PR_MODULE_GRABBER], () => {
                    });
                });

                this.ordersSerial.sendOrder(this.actuatorCommands.AX12_ONE, [this.actuatorCommands.PR_MODULE_DUMMY], () => {
                    this.ordersSerial.sendOrder(this.actuatorCommands.AX12_INIT, [this.actuatorCommands.PR_MODULE_DUMMY], () => {
                    });
                });
            }
        }
        this.logger.debug("TODO: parse command : " + receivedCommand);
    }

    /**
     * This function is reserved for the pr.
     * Open the grabber, movement to the floor. Make sure to open the module arm first.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    openGrabber(callback) {
        if (this.stdStreamConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.AX12_ONE, [this.actuatorCommands.PR_MODULE_GRABBER], callback);
        } else {
            this.logger.error("TODO: AX12 real openGrabber()");
        }
    }

    /**
     * This function is reserved for the pr.
     * Close the grabber, movement to the robot.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    closeGrabber(callback) {
        if (this.stdStreamConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.AX12_INIT, [this.actuatorCommands.PR_MODULE_GRABBER], callback);
        } else {
            this.logger.error("TODO: AX12 real closeGrabber()");
            // this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the left.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyLeft(callback) {
        if (this.stdStreamConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.AX12_TWO, [this.actuatorCommands.PR_MODULE_DUMMY], callback);
        } else {
            this.logger.error("TODO: AX12 real sendDummyLeft()");
            // this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the right.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyRight(callback) {
        if (this.stdStreamConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.AX12_ONE, [this.actuatorCommands.PR_MODULE_DUMMY], callback);
        } else {
            this.logger.error("TODO: AX12 real sendDummyRight()");
            // this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the center.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyCenter(callback) {
        if (this.stdStreamConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.AX12_INIT, [this.actuatorCommands.PR_MODULE_DUMMY], callback);
        } else {
            this.logger.error("TODO: AX12 real sendDummyCenter()");
            // this.logger.error("Serial port not connected...");
        }
    }

    /**
     * Stop the ax12 (turns it off).
     */
    stop() {
        // this.logger.error("TODO: AX12 real stop()");
        if (this.stdStreamConnected && !!this.ordersSerial) {
            this.ordersSerial.sendOrder(this.actuatorCommands.HALT, []);
        } else {
            this.logger.error("TODO: AX12 real stop()");
            // this.logger.error("Serial port not connected...");
        }

        this.logger.info("AX12 real stopped");
    }
}

// Exports an object to be sure to have a single instance in the system
module.exports = function () {
    return new Ax12Real();
};
