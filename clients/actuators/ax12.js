/**
 *  @file       Describes the ax12 module.
 *  @date       14/05/2017
 *  @module     clients/actuators/ax12
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * Logger.
 */
const Log4js = require('log4js');

/**
 * Abstract class defining the interface to interact with ax12 program.
 * The Ax12 class defines high level functions to interact with th ax12 program. This program will effectively send orders to ax12 using serial interface.
 * @abstract
 */
class Ax12 {
    /**
     * This class is abstract,do not instantiate it directly.
     */
    constructor() {
        // This is an abstract class, throw an error if it is directly instantiated or if missing virtual functions
        if (this.constructor === Ax12) {
            throw new TypeError("Cannot construct Abstract instances directly");
        }
        this.logger = Log4js.getLogger("ax12");
    }

    /**
     * This function is reserved for the pr.
     * Open the grabber, movement to the floor. Make sure to open the module arm first.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    openGrabber(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Close the grabber, movement to the robot.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    closeGrabber(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the left.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyLeft(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the right.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyRight(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the center.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyCenter(callback) {
    }

    /**
     * Stop the ax12 (turns it off).
     */
    stop() {
    }
}

module.exports = Ax12;