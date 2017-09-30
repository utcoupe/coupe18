/**
 *  @file       Describes the orders serial module.
 *  @date       13/05/17
 *  @module     clients/shared/ordersserial
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * Need the mother class for inheritance
 * @type {OrdersManager}
 */
const OrdersManager = require('./orders.manager');

/**
 * OrdersSerial is an inherited class from OrdersManager, dealing with serial communication.
 * For more information about the behaviour of OrdersManager, see OrdersManager class.
 * @augments OrdersManager
 */
class OrdersSerial extends OrdersManager {
    /**
     * Constructor of OrdersSerial, instantiate an OrdersManager class dealing with serial communication.
     * For more details, see the OrdersManager constructor.
     * @param {any} communicationLine       The communication stream, has to be a serial communication object.
     *                                      Just be sure to create a child class of OrderManager handling the type of communication line.
     * @param {function} callbackToIa       A callback used to send directly data to the IA. OrdersManager is not a client of the system,
     *                                      so the object using OrdersManager has to be a client, because OrdersManager needs to send data
     *                                      to the IA, through the websocket communication facility.
     */
    constructor(communicationLine, callbackToIa) {
        super(communicationLine, callbackToIa);

        // We replace the callback set by the Servo class
        this.comLine.on("data", function(data){
            this.parseCommand(data.toString());
        }.bind(this));
    }

    /**
     * This method sends the order parameter through a serial communication line.
     * @override
     * @protected
     * @param {string} order    The formatted order to send
     */
    comLineSend(order) {
        this.comLine.write(order);
    }
}

// Exports an object to be sure to have a single instance in the system
module.exports = function(communicationLine, callbackToIa) {
    return new OrdersSerial(communicationLine, callbackToIa);
};
