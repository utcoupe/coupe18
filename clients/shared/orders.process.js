/**
 *  @file       Describes the orders process module.
 *  @date       13/05/17
 *  @module     clients/shared/ordersprocess
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * Need the mother class for inheritance
 * @type {OrdersManager}
 */
const OrdersManager = require('./orders.manager.js');

/**
 * OrdersProcess is an inherited class from OrdersManager, dealing with standard process communication (stdin and stdout).
 * For more information about the behaviour of OrdersManager, see OrdersManager class.
 * @augments OrdersManager
 */
class OrdersProcess extends OrdersManager {
    /**
     * Constructor of OrdersProcess, instantiate an OrdersManager class dealing with standard process communication (stdin and stdout).
     * For more details, see the OrdersManager constructor.
     * @param {any} communicationLine       The communication stream, has to be a ByLine object (to read and write on stin / stdout).
     */
    constructor(communicationLine) {
        super(communicationLine);

        // We replace the callback set by the Servo class
        this.comLine.out.on("data", function (data) {
            this.parseCommand(data);
        }.bind(this));
    }

    /**
     * This method sends the order parameter through a standard process communication line.
     * @override
     * @protected
     * @param {string} order    The formatted order to send
     */
    comLineSend(order) {
        this.comLine.in.write(order);
    }
}

// Exports an object to be sure to have a single instance in the system
module.exports = function (communicationLine) {
    return new OrdersProcess(communicationLine);
};
