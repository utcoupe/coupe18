/**
 *  @file       Describes the ax12 simulation module.
 *  @date       14/05/2017
 *  @module     clients/actuators/ax12simu
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
 * Implement the Ax12 class for simulated robots.
 * As the ax12 are not simulated, this class just set timeouts before responding that the action has been made.
 * The class Ax12Simu is a singleton, there is only one instance of it in the system.
 * @augments Ax12
 */
class Ax12Simu extends Ax12 {
    /**
     * Instantiate the Ax12 class for simulation.
     * This object does nothing else, as servo motors are not simulated.
     */
    constructor() {
        super();
    }

    /**
     * This function is reserved for the pr.
     * Open the grabber, movement to the floor. Make sure to open the module arm first.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    openGrabber(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Close the grabber, movement to the robot.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    closeGrabber(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the left.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyLeft(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the right.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyRight(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Put the dummy arm (the one in the back of the robot) to the center.
     * Please make sure that a module is not engage before moving dummy.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    sendDummyCenter(callback) {
        setTimeout(callback, 200);
    }

    /**
     * Stop the ax12 (turns it off).
     */
    stop() {
        this.logger.info("AX12 simu stopped");
    }
}

// Exports an object to be sure to have a single instance in the system
module.exports = function () {
    return new Ax12Simu();
};