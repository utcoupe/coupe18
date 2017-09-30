/**
 *  @file       Describes the factory module.
 *  @date       13/05/2017
 *  @module     clients/shared/factory
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * Logger.
 */
const Log4js = require('log4js');
/**
 * This module is used to communicate with Arduino boards, in serial format.
 */
const SerialPort = require('serialport');
/**
 * This module is used to normalize path (use absolute and not relative path).
 */
const Path = require('path');
/**
 * This module is used to spawn C++ process (for ax12).
 */
const ChildProcess = require('child_process');
/**
 * This module is used to communicate with spawned process, using std/stdout stream.
 */
const Byline = require('byline');

/**
 * The factory is unique in the system.
 * In the system, some objects can be real or simulated.
 * The factory is used to build those objects, abstracting the real type of the object from the system.
 * The objects build by the factory are asserv, ax12 or servo.
 * By default, if a real object is detected, builds the real object, otherwise builds the simulated one.
 */
class Factory {
    /**
     * The constructor builds the factory and launches the detection process (to determine if the components are real or simulated).
     * As the detection takes some time, the factory needs a callback, which will be called when the initialization of the factory is done.
     * @param robot {robot}         The robot object using the factory
     * @param callback {function}   Callback to call when the factory is initialized
     */
    constructor(robot, callback) {
        this.logger = Log4js.getLogger("factory");
        this.robot = robot;
        this.robotName = this.robot.getName();
        // When factory is ready, call this callback, this is mandatory because of node async way to work
        this.factoryReadyCallback = callback;
        // Map of all devices found ("device_name","port")
        this.devicesPortMap = [];
        // List of opened serial port, waiting data to be received to determine if device detected
        this.openedSerialPort = [];
        // Flag to know if the factory is ready, if not, can't build any object
        this.factoryReady = false;
        // Launch the detection process
        this.detectArduino(() => {
            //todo do not launch both at the same time to avoid /dev/ttyX conflicts
            // When detection is done, close all serial ports to avoid conflicts with ax12 search
            this.closeAllPorts();
            this.detectAx12(() => {
                if (this.factoryReadyCallback !== undefined) {
                    this.factoryReady = true;
                    this.factoryReadyCallback();
                }
            });
        });

    }

    /**
     * Function to force building a simulated object, even if real object detected.
     * @todo implement it
     * @param type {string} Type of the object to build in simulated mode
     */
    forceSimulation(type) {
        this.logger.info("Force simulation of " + type + " (not active yet)");
    }

    /**
     * Create the object type corresponding to the parameter.
     * If the factory is not ready, does build any object.
     * If the factory is ready and if the factory as detected a real object, builds and returns the real object.
     * Otherwise, the function returns the simulated object.
     * @public
     * @param type {string}         Type of the object (asserv, ax12 or servo)
     * @returns {ax12|asserv|servo} Type object (real if real detected, simulated otherwise)
     */
    createObject(type) {
        var returnedObject;
        if (this.factoryReady) {
            switch (type) {
                case "asserv" : {
                    if (this.devicesPortMap[this.robotName + "_asserv"] !== undefined) {
                        this.logger.info("Asserv is real, arduino detected on port : " + this.devicesPortMap[this.robotName + "_asserv"]);
                        returnedObject = require('../asserv/asserv.real')(this.robot, this.devicesPortMap[this.robotName + "_asserv"]);
                    } else {
                        this.logger.fatal("Asserv is simu");
                        returnedObject = require('../asserv/asserv.simu')(this.robot);
                    }
                    break;
                }
                case "servo" : {
                    if (this.devicesPortMap[this.robotName + "_others"] !== undefined) {
                        this.logger.info("Servo is real, arduino detected on " + this.devicesPortMap[this.robotName + "_others"]);
                        returnedObject = require('../actuators/servo.real')(this.robot, this.devicesPortMap[this.robotName + "_others"]);
                    } else {
                        this.logger.fatal("Servo is simu");
                        returnedObject = require('../actuators/servo.simu')(this.robot);
                    }
                    break;
                }
                case "ax12" : {
                    this.logger.debug("Ax12 not tested yet !");
                    if (this.devicesPortMap["ax12"] !== undefined) {
                        this.logger.info("AX12 is real, usb2ax detected");
                        returnedObject = require('../actuators/ax12.real')(this.robot, this.devicesPortMap["ax12"]);
                    } else {
                        this.logger.fatal("AX12 is simu");
                        returnedObject = require('../actuators/ax12.simu')(this.robot);
                    }
                    break;
                }
                default:
                    this.logger.error("Type " + type + " can not be build by the factory, it does not exists");
            }
        } else {
            this.logger.error("Factory is not ready to build " + type);
        }
        return returnedObject;
    }

    /**
     * Arduino detection process.
     * Arduino boards are connected through serial port, but don't know which one. So the detection opens all serial ports
     * detected on the system and waits the arduino to send it's id.
     * When an arduino is detected, add it and the corresponding serial port in the devicesPortMap map.
     * When the detection times out, call the callback.
     * @private
     * @param callback {function}   Callback to call when the ax12 detection is done.
     */
    detectArduino(callback) {
        this.logger.info("Detecting Arduinos...");
        setTimeout(callback.bind(this), 5000);
        SerialPort.list(function (err, ports) {
            // Open each listed serial port and add a callback to detect if it is an arduino
            for (var currentPort in ports) {
                this.openedSerialPort[currentPort] = new SerialPort(ports[currentPort].comName, {
                    baudrate: 57600,
                    parser: SerialPort.parsers.readline('\n')
                });
                this.openedSerialPort[currentPort].on("data", function (currentPort, data) {
                    // this.logger.debug(data.toString());
                    if (data.toString().indexOf(this.robotName + "_asserv") != -1) {
                        this.logger.info("Real asserv detected");
                        this.devicesPortMap[this.robotName + "_asserv"] = ports[currentPort].comName;
                    }
                    else if (data.toString().indexOf(this.robotName + "_others") != -1) {
                        this.logger.info("Real others detected");
                        this.devicesPortMap[this.robotName + "_others"] = ports[currentPort].comName;
                    } else {
                        this.logger.info("Unknown reception : " + data.toString());
                    }
                }.bind(this, currentPort));
            }
        }.bind(this));
    }

    /**
     * AX12 detection process.
     * The process first spawn the AX12 process. It will check if it detects an ax12 connected through the USB2AX module.
     * When an ax12 is dtected or the detection times out, the process ends and the callback is called.
     * If an ax12 is detected, add the information in the devicesPortMap map.
     * @private
     * @param callback {function}   Callback to call when the ax12 detection is done.
     */
    detectAx12(callback) {
        this.logger.info("Detecting AX12...");
        // Launch a timeout process to end the detection if no ax12 is detected.
        var detectionTimeout = setTimeout(() => {
            this.logger.info("AX12 detection timeout");
            // Kill the ax12 Ax12Program, has to be started by the extension using the ax12 Ax12Program
            ax12.kill();
            callback();
        }, 1000);
        // Spawn the ax12 process
        var ax12 = ChildProcess.spawn(Path.normalize(process.env["UTCOUPE_WORKSPACE"] + "/bin/ax12"));
        // Set the error callback, for instance if the ax12 binary is not present
        ax12.on('error', function (err) {
            if (err.code === 'ENOENT') {
                this.logger.fatal("ax12 Ax12Program executable not found! Is it compiled ? :) Was looking in \"" + Path.resolve(Path.normalize(process.env["UTCOUPE_WORKSPACE"] + "/bin/ax12")) + "\"");
                process.exit();
            }
            this.logger.error("c++ subprocess terminated with error:", err);
            console.log(err);
        }.bind(this));
        // Set the exit callback to ensure that ax12 process is killed
        process.on('exit', function () {
            if (ax12.connected) { //and was still connected (dont kill another process)
                ax12.kill();
            }
        });
        // Create the stream to dialog with the ax12 process, using standard stdin and stdout
        var stdout = Byline.createStream(ax12.stdout);
        stdout.setEncoding('utf8');
        stdout.on('data', function (data) {
            // Waiting to receive "ax12" string, telling that an ax12 has been detected
            if (data.indexOf("ax12") == 0) {
                this.devicesPortMap["ax12"] = "I don't know";
                clearTimeout(detectionTimeout);
                // ax12 killed if detected
                ax12.kill();
                this.logger.info("AX12 detection end");
                callback();
            }
        }.bind(this));
        ax12.stderr.on('data', function (data) {
            // this.logger.error("stderr :"+data.toString());
        }.bind(this));
    }

    /**
     * Close all serial ports opened by the detection process.
     * @private
     */
    closeAllPorts() {
        for (var port in this.openedSerialPort) {
            this.openedSerialPort[port].close();
        }
    }
}

// Export an object, to make it unique in the system
module.exports = function (robotName, callback) {
    return new Factory(robotName, callback);
};
