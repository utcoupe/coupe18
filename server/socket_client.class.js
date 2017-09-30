/**
 * Socket Client
 * 
 * @module server/socket_client
 * @requires log4js
 * @requires socket.io-client
 * @see {@link server/socket_client.SocketClient}
 */

"use strict";

var SocketIO_client = require('socket.io-client');
const Log4js = require('log4js');

/**
 * logger
*/
var logger = Log4js.getLogger('Client');

/**
 * Socket permettant l'échange entre les clients et le serveur
 * 
 */
class SocketClient {
	constructor (params) {
		
		/**
		 * Server IP
		 * @type {string}
		 */
		this.server_ip = params.server_ip || '127.0.0.1:3128';
		/**
		 * Socket du client
		 * @type {socket.io-client}
		 */
		this.client = SocketIO_client('http://'+this.server_ip);
		/**
		 * callbacks
		 * @type {Array<Function>}
		 */
		this.callbacks = {};
		/**
		 * Variable permettant au client d'ignorer les échanges réseaux
		 * @type {bool}
		 */
		this.muted = false;

		if(!!params.type)
			/** Type of client */
			this.type = params.type;
		else
			logger.error("Missing client type.");

		// When the client is connected to the server
		this.client.on('connect', () => {
			// logger.info('Client connected to server');
			this.client.emit('type', {
				type: this.type,
				options: {
					name: this.type
				}
			});
			if(!!this.callbacks.connect)
				this.callbacks.connect();
			// this.client.emit('order', {to:'client2',text:'Hello!'});
		});

		// When the client is connected to the server
		this.client.on('disconnect', () => {
			logger.error('Client disconnected from server');
		});

		// When the client receive log from the server
		this.client.on('log', (data) => {
			logger.info('[Server log] '+data);
		});

		// When the client receive order from the server
		this.client.on('order', (data) => {
			// logger.info('[Order to '+data.to+'] '+data.text);
			// On n'autorise pas la réception de message si on est muet, sauf si c'est un start
			if (this.muted && data.name != "start" && data.name != "kill")
				logger.info("A client tried to receive an order, but it is muted!");
			else
				if(!!this.callbacks.order)
					if (!!data.name)
						this.callbacks.order(data.from, data.name, data.params || {});
					else
						logger.error("Order has no name ! : " + data);
		});

		// If after 500ms the client isn't connected, throw "server not found" error
		setTimeout(() => {
			if(this.client.disconnected)
				this.errorServerNotFound();
		}, 500);
	}

	/**
	 * connect
	 * 
	 * @param {function} callback
	 */
	connect (callback) {
		this.callbacks.connect = callback;
	}

	/**
	 * order
	 * 
	 * @param {function} callback
	 */
	order (callback) {
		this.callbacks.order = callback;
	}

	/**
	 * Send parameters
	 * 
	 * @param {string} to
	 * @param {string} name
	 * @param {Object} params
	 */
	send (to, name, params) {
		// logger.debug('send %s to %s', name, to);
		if (this.muted)
			logger.info('A client tried to send an order, but he is muted!');
		else
			this.client.emit('order', {
				to: to,
				name: name,
				params: params,
				from: this.type
			});
	}

	// Error functions
	/**
	 * Appends the error message in the logger
	 * 
	 * @param {string} msg message to send
	 */
	throwError (msg) {
		logger.error(msg);
	}

	/**
	 * Throw Server not found
	 */
	errorServerNotFound () {
		this.throwError('Server not found at '+this.server_ip+', please make sure the server is running.');
	}

	/**
	 * Throw Server timed out
	 */
	errorServerTimeout () {
		this.throwError('Server timed out, please make sure the server is still running.');
	}

	/**
	 * Désactive temporairement le client
	 */
	mute () {
		this.muted = true;
	}

	/**
	 * Réactive le client
	 */
	unMute () {
		this.muted = false;
	}
}

module.exports = SocketClient;
