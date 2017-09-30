/**
 *  @file       Describes the servo module.
 *  @date       13/05/2017
 *  @module     clients/actuators/servo
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * Logger.
 */
const Log4js = require('log4js');

/**
 * Abstract class defining the interface to interact with servo motors.
 * The Servo class is a high vision of the way the robots servo motors are working. The goal of this class is to provide
 * high level functions to send orders to the servo motors board.
 * @abstract
 */
class Servo {
    /**
     * Create the logger and build the Servo object.
     * This class is abstract,do not instantiate it directly.
     * @param robot {Robot} The robot to handle the servo motors
     */
    constructor(robot) {
        // This is an abstract class, throw an error if it is directly instantiated or if missing virtual functions
        if (this.constructor === Servo) {
            throw new TypeError("Cannot construct Abstract instances directly");
        }
        this.logger = Log4js.getLogger("servo");
        this.robotName = robot.getName();
    }

    /**
     * Stop sending orders to the servo motors.
     */
    stop() {
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
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the open position.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmOpen(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the init position.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmInit(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Engage a module, put it in position to make it rotate.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleEngage(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Drop a module.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleDrop(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Put the arm which takes the modules in the init position.
     * @public
     * @param callback {function}   Function to call when the action is done
     * @param params {Object}       Parameters for the rotation, as the target color
     */
    moduleRotate(callback, params) {
    }

    /**
     * This function is reserved for the pr.
     * Make the wheel on the arm to take the module start spinning.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmStartRotate(callback) {
    }

    /**
     * This function is reserved for the pr.
     * Make the wheel on the arm to take the module stop spinning.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    moduleArmStopRotate(callback) {
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
    }

    /**
     * This function is reserved for the gr.
     * Turn off the canon, the wheels will start to spin.
     * Make sure to call the CloseTrunk function to close the trunk, in order to avoid dropping balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOffCanon(callback) {
    }

    /**
     * This function is reserved for the gr.
     * Turn on the sweeper, which is the actuator grabbing the balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOnSweeper(callback) {
    }

    /**
     * This function is reserved for the gr.
     * Turn off the sweeper, which is the actuator grabbing the balls.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    turnOffSweeper(callback) {
    }

    /**
     * This function is reserved for the gr.
     * Launch the rocket, which is the funny action.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    launchRocket(callback) {
    }

    /**
     * This function is reserved for the gr.
     * Open the trunk, which is the door of the canon, so make sure to open it before shooting.
     * To make the shoot work, do not forget to call the turnOnCanon function.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    openTrunk(callback) {
    }

    /**
     * This function is reserved for the gr.
     * Close the trunk, which is the door of the canon.
     * Closing the trunk will not turn off the canon, use the turnOffCanon to turn it off.
     * @public
     * @param callback {function}   Function to call when the action is done
     */
    closeTrunk(callback) {
    }
}

module.exports = Servo;