/**
 *  @file       Describes the servo real module.
 *  @date       13/05/2017
 *  @module     clients/actuators/servoreal
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * Mother class, mandatory for inheritance.
 * @type {Servo}
 */
const Servo = require('./servo');
/**
 * The defineParser module is used to convert C header files #define in an object usable in js.
 */
const defineParser = require("../shared/defineparser");
/**
 * As the servo real communicate with servo motors boards in serial, the serial port object is used.
 * @type {SerialPort}
 */
const SerialPort = require('serialport');

/**
 * Implement the Servo class for real robots.
 * The communication to servo boards is based on the serial protocol, the serial connection is created by ServoReal.
 * This class is the interface between the js system (using the public methods of the Servo abstract class) and the embedded one.
 * The communication protocol is defined as follow : order;id;arg1;arg2;argn.
 * The orders list is parsed from the header file corresponding to the robot, using the defineParser object.
 * The response of the servo board can be of 2 types : an order response (beginning with order_id;) or a debug string.
 * The ServoReal class is unique in the system, all other class which need to deal with servo has to use this class to do it.
 * Before being able to send orders to the servo board, this class will check if the board is connected and responding,
 * so it may takes a few seconds before the ServoReal object is ready.
 * The orders received from the servo board are handled by the OrdersSerial object.
 * @augments Servo
 */
class ServoReal extends Servo {
    /**
     * Build the ServoReal object, which is unique in the system.
     * ServoReal handles the serial communication with the servo motor boards, it just needs to know on which serial port the board is connected.
     * It loads the orders available using the defineParser object.
     * @param robot {Robot}         The robot to handle the servo motors
     * @param serialPort {string}   The serial port to use to communicate with the servo board (with respect with /dev/ttyX format)
     */
    constructor(robot, serialPort) {
        super(robot);
        this.actuatorCommands = defineParser(process.env.UTCOUPE_WORKSPACE + "/arduino/" + this.robotName + "/others/protocol.h");
        this.ordersSerial = undefined;
        // Connected means that the node has started the device through serial port
        this.serialPortConnected = false;
        this.serialPort = new SerialPort(serialPort, {
            baudrate: 57600,
            parser: SerialPort.parsers.readline("\n")
        });
        this.serialPort.on("data", function (data) {
            this.parseCommand(data.toString());
        }.bind(this));
        this.serialPort.on("error", function (data) {
            this.logger.error("Serial port error : " + data.toString());
        }.bind(this));
        this.serialPort.on("close", function () {
            this.serialPortConnected = false;
            //todo ?
            // this.sendStatus();
            this.logger.debug("Serial port close");
        }.bind(this));
    }

    /**
     * Function called when a string is received on the serial port, which means a reception from the servo board.
     * At the first reception, ask the servo board to start.
     * When an ack of start is received, initialize all the servo motors to know that they are all working fine.
     * The commands received are handled by the OrdersSerial object (this object changes the "data" callback to call,
     * so this function will only be called once when the servo board has started).
     * @private
     * @param receivedCommand {string}  Command received, with respect to the servo board communication protocol
     */
    parseCommand(receivedCommand) {
        if (!this.serialPortConnected) {
            // If not connected, wait the ID of the arduino before doing something else
            if (receivedCommand.indexOf(this.robotName + "_others") == 0) {
                //todo find a way to make it proper
                var order = [this.actuatorCommands.START, 0].join(";") + ";\n";
                this.logger.debug(order);
                this.serialPort.write(order);
            } else {
                this.logger.debug(receivedCommand.toString());
                // Trigger on reception of ack from arduino that it has started
                if (receivedCommand.indexOf("0;") == 0) {
                    this.serialPortConnected = true;
                    // Use the OrdersSerial object to handle orders, this object will replace this callback by it's own.
                    this.ordersSerial = require("../shared/orders.serial")(this.serialPort);
                    this.logger.debug("INIT movements");
                    if (this.robotName == "pr") {
                        this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.PR_MODULE_ARM_ROTATE], () => {
                            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.PR_MODULE_ARM_ROTATE], () => {
                            });
                        });
                        this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.PR_MODULE_ARM], () => {
                            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.PR_MODULE_ARM], () => {
                            });
                        });

                        this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.PR_MODULE_DROP_R], () => {
                            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.PR_MODULE_DROP_R], () => {
                            });
                        });
                        this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.PR_MODULE_DROP_L], () => {
                            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.PR_MODULE_DROP_L], () => {
                            });

                        });
                    } else {
                        this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.GR_CANON], () => {
                            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.GR_CANON], () => {
                            });
                        });

                        this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.GR_SWEEPER], () => {
                            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.GR_SWEEPER], () => {
                            });
                        });

                        this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.GR_LOADER], () => {
                            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.GR_LOADER], () => {
                            });
                        });
                    }
                }
            }
        }
    }

    /**
     * Stop sending orders to the servo motors.
     * Stop the serial communication.
     * @public
     */
    stop() {
        if (this.serialPort.isOpen()) {
            this.ordersSerial.sendOrder(this.actuatorCommands.HALT, function () {
                this.serialPort.close();
                this.logger.info("Asserv real has stopped");
            }.bind(this));
        }
        this.serialPortConnected = false;
    }

    ////////////////////////////////
    //          PR ORDERS         //
    ////////////////////////////////

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the close position.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmClose(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.PR_MODULE_ARM], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the open position.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmOpen(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.PR_MODULE_ARM], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the init position.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmInit(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_INIT, [this.actuatorCommands.PR_MODULE_ARM], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Engage a module, put it in position to make it rotate.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleEngage(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.PR_MODULE_DROP_R], callback);
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.PR_MODULE_DROP_L], () => {
            });
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Drop a module.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleDrop(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.PR_MODULE_DROP_R], callback);
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.PR_MODULE_DROP_L], () => {
            });
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the init position.
     * @public
     * @param callback {function}   Function to call when the action is done
     * @param params {Object}       Parameters for the rotation, as the target color
     */
    moduleRotate(callback, params) {
        if (this.serialPortConnected) {
            var color_number = 0;
            if (params.color == "yellow") {
                color_number = 2;
            } else if (params.color == "blue") {
                color_number = 1;
            } else if (params.color == "null") {
                color_number = 0;
            } else {
                this.logger.error("Rotate color " + params + " does not exists");
            }
            this.ordersSerial.sendOrder(this.actuatorCommands.MODULE_ROTATE, [color_number], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Make the wheel on the arm to take the module start spinning.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmStartRotate(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.PR_MODULE_ARM_ROTATE], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the pr.
     * Make the wheel on the arm to take the module stop spinning.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmStopRotate(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.PR_MODULE_ARM_ROTATE], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    ////////////////////////////////
    //          GR ORDERS         //
    ////////////////////////////////

    /**
     * This function is reserved for the gr.
     * Turn on the canon, the wheels will start to spin.
     * Make sure to call the OpenTrunk function to open the trunk, otherwise the robot will not be able to shoot.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOnCanon(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.GR_CANON], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the gr.
     * Turn off the canon, the wheels will start to spin.
     * Make sure to call the CloseTrunk function to close the trunk, in order to avoid dropping balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOffCanon(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.GR_CANON], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the gr.
     * Turn on the sweeper, which is the actuator grabbing the balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOnSweeper(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.GR_SWEEPER], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the gr.
     * Turn off the sweeper, which is the actuator grabbing the balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOffSweeper(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.GR_SWEEPER], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the gr.
     * Launch the rocket, which is the funny action.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    launchRocket(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.GR_ROCKET], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the gr.
     * Open the trunk, which is the door of the canon, so make sure to open it before shooting.
     * To make the shoot work, do not forget to call the turnOnCanon function.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    openTrunk(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_OPEN, [this.actuatorCommands.GR_LOADER], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }

    /**
     * This function is reserved for the gr.
     * Close the trunk, which is the door of the canon.
     * Closing the trunk will not turn off the canon, use the turnOffCanon to turn it off.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    closeTrunk(callback) {
        if (this.serialPortConnected) {
            this.ordersSerial.sendOrder(this.actuatorCommands.SERVO_CLOSE, [this.actuatorCommands.GR_LOADER], callback);
        } else {
            this.logger.error("Serial port not connected...");
        }
    }
}

// Exports an object to be sure to have a single instance in the system
module.exports = function (robot, serialPort) {
    return new ServoReal(robot, serialPort);
};
