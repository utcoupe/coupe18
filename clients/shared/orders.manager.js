/**
 *  @file       Describes the orders manager abstract module.
 *  @date       13/05/17
 *  @module     clients/shared/ordersmanager
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * Logger.
 */
const Log4js = require('log4js');

/**
 * OrdersManager is an abstract class defining the interface to manage orders in the system.
 * This class is used to handle orders using the UTCoupe format for communication with external components (actuators).
 * The general format is : order_name;order_id;order_parameter1;...
 * The OrdersManager class is the unique way, in the system, to communicate with actuators. This class handles the reception and the sending of data.
 * This kind of communication is used to communicate with arduino boards (serial communication) and the ax12 program (inter-process communication).
 * In details, when a message is send by OrdersManager, the order_id is generated (to be sure it's unique) and save in a structure
 * with the callback to call when the actuator sends an ack of the order.
 * This module does not check if the messages to send are well formatted to be interpreted by the actuator component.
 * @abstract
 */
class OrdersManager {
    /**
     * Creates an instance of OrdersManager (the class is abstract, do not use it directly).
     * The order_id start from 1, because the id 0 is reserved for the device identification (done by the factory).
     * The OrdersManager works with a communication line, this stands for a stream which can be used by the OrdersManager
     * to send and receive data. As this communication line isn't created by the OrdersManager, be sure that the component
     * using it will close it.
     * @param {any} communicationLine       The communication stream, can be a serial object, an inter-process object, ...
     *                                      Just be sure to create a child class of OrderManager handling the type of communication line.
     * @param {function} callbackSendToIa   A callback used to send directly data to the IA. OrdersManager is not a client of the system,
     *                                      so the object using OrdersManager has to be a client, because OrdersManager needs to send data
     *                                      to the IA, through the websocket communication facility.
     */
    constructor(communicationLine, callbackSendToIa) {
        // This is an abstract class, throw an error if it is directly instantiated or if missing virtual functions
        if (this.constructor === OrdersManager) {
            throw new TypeError("Cannot construct Abstract instances directly");
        }
        this.logger = Log4js.getLogger("orders_manager");
        if (callbackSendToIa !== undefined) {
            this.sendToIa = callbackSendToIa;
        } else {
            this.logger.warn("No callbackSendToIa given..." + callbackSendToIa);
        }
        // IDs starts from 1, 0 is reserved
        this.currentId = 1;
        // The structure storing the orders_id and the associated callback
        this.ordersCallback = [];
        this.comLine = communicationLine;
        // ComLine connected means that we can send orders
        // Connected by default if comLine is defined
        this.comLineConnected = false;
        if (communicationLine !== undefined) {
            this.comLineConnected = true;
        }
    }

    /**
     * Call the callback corresponding to the received order id, stored in ordersCallback.
     * If the id does not exists, just send a message to advertise the system.
     * @private
     * @param {Integer} orderId         The id of the received order
     * @param {array<string>} params    The parameters received, associated to the order
     */
    callOrderCallback(orderId, params) {
        for (var index = 0; index < this.ordersCallback.length; index++) {
            if (this.ordersCallback[index][0] == orderId) {
                // Call the callback
                if (this.ordersCallback[index][1] != null) {
                    this.logger.debug("Callback for order " + orderId);
                    this.ordersCallback[index][1](params);
                } else {
                    this.logger.error("Callback for order " + orderId + " is null...");
                }
                // Remove the line from the array
                this.ordersCallback.splice(index, 1);
            }
        }
    }

    /**
     * Send an order using through the communication line.
     * This method automatically adds the generated id of the order and store it with the callback in the ordersCallback structure.
     * @public
     * @param {char} orderType          Character defining the order
     * @param {array<string>|null} args Array of string, list of arguments associated with the order
     * @param {function} callback       Callback to call when an ack that the order has been executed is received
     */
    sendOrder(orderType, args, callback) {
        if (this.comLineConnected) {
            this.logger.debug("args = " + args);
            args = args || [];
            var order = [orderType, this.currentId].concat(args).join(";") + ";\n";
            this.ordersCallback.push([this.currentId, callback]);
            this.logger.debug("Send order : " + order);
            this.comLineSend(order);
            this.currentId++;
        } else {
            this.logger.error("Communication line is not connected...");
        }
    }

    /**
     * Parse a received command to extract the order_id.
     * Once the order_id is found, call the associated callback (if it exists).
     * If no order_id is found, assume that the received command is a debug string, so display it.
     * @protected
     * @param {string} receivedCommand  Raw received string, to be parsed
     */
    parseCommand(receivedCommand) {
        // Check if the received command is a debug string or a response from an order
        // As the received command is a string, the ; index is depending on the number of digits of the order id received
        if (receivedCommand.indexOf(";") != -1) {
            // It's an order response
            var splittedCommand = receivedCommand.split(";");
            // Do not remove, mandatory to debug asserv
            //todo find a better way to do it...
            if (splittedCommand[0] == "~") {
                this.logger.info(splittedCommand[2], splittedCommand[3], splittedCommand[4], splittedCommand[5], splittedCommand[6], splittedCommand[7], splittedCommand[8], splittedCommand[9], splittedCommand[10]);
                this.logger.info("PID (FP!): ", splittedCommand[11] / 1000, splittedCommand[12] / 1000, splittedCommand[13] / 1000);
                if (this.sendToIa !== undefined) {
                    this.sendToIa(splittedCommand[2], splittedCommand[3], splittedCommand[4]);
                }
            } else {
                this.callOrderCallback(parseInt(splittedCommand[0]), splittedCommand.slice(0, 2));
            }
        } else {
            // It's a debug string
            this.logger.debug(receivedCommand.toString());
        }
    }

    /**
     * This method effectively sends an order using the communication line way to send data.
     * The order is formatted by the sendOrder method.
     * Please be sure to override this method in inherited class.
     * @abstract
     * @protected
     * @param {string} order    The formatted order to send
     */
    comLineSend(order) {
    }
}

module.exports = OrdersManager;
