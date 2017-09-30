/**
 *  @file       Describes the client module.
 *  @date       01/04/2017
 *  @module     clients/shared/client
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * Logger.
 */
const Log4js = require('log4js');
/**
 * Config file, containing IP and port of the server.
 */
const CONFIG = require('../../config.js');
/**
 * Socket client object for communication.
 * @type {SocketClient}
 */
const SocketClient = require('../../server/socket_client.class.js');

/**
 * Client is an abstract class defining the interface and the common functions to be a client in the UTCoupe system.
 * A client is the only object able to send and receive messages form the other clients of the system.
 * @abstract
 */
class Client {
    /**
     * The constructor creates the logger and the socketClient object, mandatory for the communication.
     * The parameters of the socket (IP and port) are loaded from the config.js file.
     * When the object is created, it automatically binds the abstract reception callback.
     * @param clientName {string}   Name of the client, used to route messages
     */
    constructor(clientName) {
        this.logger = Log4js.getLogger(clientName);
        this.clientName = clientName;
        this.parser = null;
        var server = CONFIG.server;
        this.client = new SocketClient({
            server_ip: server,
            type: clientName
        });
        // Bind the callback function, which has to be overloaded
        this.client.order(this.takeOrder.bind(this));
    }

    /**
     * Abstract callback function called when a client receives a message.
     * This function has to be used to parse and dispatch the received message.
     * @abstract
     * @protected
     * @param from {string} Name of the client which has send the message
     * @param name {string} Name of the order
     * @param param {json}  Parameters of the order
     */
    takeOrder(from, name, param) {
        throw new TypeError("client:takeOrder is abstract !");
    }

    /**
     * Send data to the IA client.
     * @protected
     * @param name {string} Name of the order
     * @param params {json} Parameters of the order
     */
    sendDataToIA(name, params) {
        this.client.send('ia', name, params);
    }

    /**
     * Start the client.
     * When a client has started, it can send and receive messages.
     * @protected
     */
    start() {
        this.client.unMute();
    }

    /**
     * Stop a client
     * When a client is stopped, it can't send and receive messages, but the client is still alive.
     * @protected
     */
    stop() {
        this.client.mute();
        this.logger.info(this.clientName + " has stopped.");
    }
}

module.exports = Client;