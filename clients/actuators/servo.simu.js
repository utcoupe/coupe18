/**
 *  @file       Describes the servo simulation module.
 *  @date       14/05/2017
 *  @module     clients/actuators/servosimu
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
 * Implement the Servo class for simulated robots.
 * As the servo motors are not simulated, this class just set timeouts before responding that the action has been made.
 * The class ServoSimu is a singleton, there is only one instance of it in the system.
 * @augments Servo
 */
class ServoSimu extends Servo {
    /**
     * Instantiate the Servo class for simulation.
     * This object does nothing else, as servo motors are not simulated.
     * @param robot {Robot} The robot to handle the servo motors
     */
    constructor(robot) {
        super(robot);
    }

    /**
     * Stop sending orders to the servo motors.
     */
    stop() {
        this.logger.info("Servo simu stopped");
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
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the open position.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmOpen(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the init position.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmInit(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Engage a module, put it in position to make it rotate.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleEngage(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Drop a module.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleDrop(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the init position.
     * @public
     * @param callback {function}   Function to call when the action is done
     * @param params {Object}       Parameters for the rotation, as the target color
     */
    moduleRotate(callback, params) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Make the wheel on the arm to take the module start spinning.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmStartRotate(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the pr.
     * Make the wheel on the arm to take the module stop spinning.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmStopRotate(callback) {
        setTimeout(callback, 200);
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
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the gr.
     * Turn off the canon, the wheels will start to spin.
     * Make sure to call the CloseTrunk function to close the trunk, in order to avoid dropping balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOffCanon(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the gr.
     * Turn on the sweeper, which is the actuator grabbing the balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOnSweeper(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the gr.
     * Turn off the sweeper, which is the actuator grabbing the balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOffSweeper(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the gr.
     * Launch the rocket, which is the funny action.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    launchRocket(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the gr.
     * Open the trunk, which is the door of the canon, so make sure to open it before shooting.
     * To make the shoot work, do not forget to call the turnOnCanon function.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    openTrunk(callback) {
        setTimeout(callback, 200);
    }

    /**
     * This function is reserved for the gr.
     * Close the trunk, which is the door of the canon.
     * Closing the trunk will not turn off the canon, use the turnOffCanon to turn it off.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    closeTrunk(callback) {
        setTimeout(callback, 200);
    }
}

// Exports an object to be sure to have a single instance in the system
module.exports = function (robot) {
    return new ServoSimu(robot);
};
