/**
 * Server module
 * @module server/server
 * @requires module:ia/main
 * @requires log4js
 * @requires child_process
 * @requires ansi-to-html
 * @requires os
 * @requires socket.io
 * @see {@link server/server.Server}
 */

"use strict";

const Log4js = require('log4js');
var Convert = require('ansi-to-html');

var logger = Log4js.getLogger('Server');
var spawn = require('child_process').spawn;
var convert = new Convert({newLine: true});
var os = require('os');

/**
 * Serveur pour toutes les communications
 * 
 */
class Server {
	/**
	 * Starts a server on the port specified. Default port: 3128
	 *
	 * @param {int} [server_port=3128] Server port
	 */
	constructor (server_port) {
		/**
		 * @type {int}
		 */
		this.server_port = server_port || 3128;

		this.verbose = false;

		/** @type {String} */
		this.ip = "127.0.0.1";

		// Get server IP address
		var networkInterfaces = os.networkInterfaces();
		try {
			var whiteList = [
				"ra0",
				"wlan0",
				"Wi-Fi",
				"wlp3s0",
				"enp4s0"
			];
			for (var net in whiteList) {
				if (!!networkInterfaces[net]){
					this.ip = networkInterfaces[net][0].address;
					break;
				}
			}
		}
		catch(e) {
			logger.warn("Error looking for ip address, fallback to loopback address.");
			this.ip = "127.0.0.1";
		}
		/** @type {string} */
		this.ip_port = this.ip+':'+this.server_port;

		/**
		 * Create the server
		 * @type {Object}
		 */
		this.server = require('socket.io')();

		/**
		 * Create the network default object
		 * @type {Object}
		 * */
		this.network = {
			server: {
				name: "Server",
				ip: this.ip_port
			},
			webclient: {},
			ia: {},
			lidar: {},
			hokuyo: {},
			pr: {},
			gr: {},
			unit_grabber: {},
			base_constructor: {},
			canon: {},
			sweeper: {},
            rocket : {}
		};
		/** @type {Object} */
		this.utcoupe = {
			'ia': false,
			'pr': false,
			'gr': false,
			'hokuyo': false
		};
		 /** @type {Object} */
		this.progs = {
			'ia': null,
			'pr': null,
			'gr': null,
			'lidar': null,
			'hokuyo': false
		}

		this.spamListLevel1 = ["lidar.all", 'utcoupe', 'simulateur', 'gr.pos', 'pr.pos'];
		this.spamListLevel2 = ["hokuyo.polar_raw_data"];


		// When the client is connected
		this.server.on('connection', (client) => {
			// When the client is disconnected
			client.on('disconnect', () => {
				logger.info(client.type+" is disconnected!");
				try {
					delete this.network[client.type][client.id];
				}
				catch(e) { }
				this.sendNetwork();
			});

			// When the client send his type
			client.on('type', (data) => {
				if(typeof data.type !== 'string') {
					logger.error("The client type sent isn't a string");
					return;
				}
				if(!(data.type in this.network)) {
					logger.error("The client type `"+data.type+"` isn't valid");
					return;
				}
				// The type is valid
				client.type = data.type;
				logger.info(client.type+" is connected!");
				data.options.ip = client.handshake.address;
				this.network[client.type][client.id] = data.options;
				// console.log(this.network);
				client.join(client.type);
				client.emit('log', "Connected to the server successfully at " + client.handshake.headers.host);
				this.sendNetwork();
				this.sendUTCoupe();
				this.sendVerbosity();
			});

			// When the client send an order
			client.on('order', (data) => {
				// console.log(data);
				if(typeof data !== 'object') {
					logger.error("The client order sent isn't a object");
					return;
				}
				if(data.to === undefined) {
					logger.error("The order hasn't 'to' argument (recipient)");
					return;
				}
				// if(!(data.to in client.adapter.rooms)) {
				// 	logger.warn("The order recipient `"+data.to+"` doesn't exist.");
				// }
				switch (data.name) {
					case 'server.spawn':
						this.spawn(data.params);
						break;
					case 'server.kill':
						this.kill(data.params);
						break;
					case 'server.childrenUpdate':
						// console.log(this.network);
						this.network[client.type][client.id].status = data.params.status || "";
						this.network[client.type][client.id].children = data.params.children || "";
						// console.log(this.network);
						this.sendNetwork();
						break;
					case 'server.iaParams':
						this.network[client.type][client.id].color = data.params.color || "";
						this.network[client.type][client.id].we_have_hats = data.params.we_have_hats;
						this.sendNetwork();
						break;
					case 'server.sync_all_git' :
						logger.info("Starting to sync all git repositories");
						spawn('/root/sync_all_git.sh', [], {
							detached: true
						});
						break;
					case 'server.flash_arduinos' :
						logger.info("Starting to flash all arduinos");
						spawn('/root/flash_all_arduinos.sh', [], {
							detached: true
						});
						break;
					case 'server.verbose':
						// Toogle verbose mode
						this.verbose = !this.verbose;
						logger.info("Have been asked to " + (this.verbose?"talk a lot :)":"shut up :/"));
						this.sendVerbosity();
						break;
					default:
						// The order is valid
						// logger.info("Data " +data.name+ " from " +data.from+ " to " +data.to);
						if (!this.verbose &&
							this.spamListLevel1.concat(this.spamListLevel2).indexOf(data.name) != -1) {
							// Verbose mode level 1 : send most of spam messages
							this.server.to(data.to).emit('order', data);
						} else if (this.spamListLevel2.indexOf(data.name) != -1) {
							// Not verbose, don't copy level 2 spam message to webclients
							this.server.to(data.to).emit('order', data);
						} else {
							// Not verbose, copy to all regular messages
							this.server.to('webclient').to(data.to).emit('order', data);
						}
				}
			});
		});

		this.server.listen(this.server_port);
		logger.info("Server started at "+this.ip_port);
	}

	/**
	 * Send Network
	 */
	sendNetwork (){
		// logger.info("Message sent to webclient !");
		// logger.info(this.network);
		this.server.to('webclient').emit('order', {
			to: 'webclient',
			name: 'reseau',
			params: {
				network: this.network
			},
			from: 'server'
			});
	}

	/**
	 * Launch the robot
	 */
	spawn (params) {
		var prog = params.prog;
		if(!this.utcoupe[prog]) {
			switch(prog) {
				case 'ia':
					this.progs[prog] = spawn('node', [process.env["UTCOUPE_WORKSPACE"] + '/ia/main.js', params.color, params.we_have_hats/*, params.nb_erobots, params.EGR_d, params.EPR_d*/]);
				break;
				case 'pr':
					//this.progs[prog] = spawn('ssh', ['igep', process.env["UTCOUPE_WORKSPACE"] + '/root/main.sh']);
                    //todo launch remote client
					this.progs[prog] = spawn('node', [ process.env["UTCOUPE_WORKSPACE"] + '/clients/main/main.pr.js']);
				break;
				case 'gr':
					this.progs[prog] = spawn('node', [ process.env["UTCOUPE_WORKSPACE"] + '/clients/main/main.gr.js']);
				break;
				case 'hokuyo':
					// this.progs[prog] = spawn('ssh', ['raspi', process.env["UTCOUPE_WORKSPACE"] + '/hokuyo/main.js']);
					this.progs[prog] = spawn('node', [ process.env["UTCOUPE_WORKSPACE"] + '/hokuyo/main.js']);
				break;
				case 'lidar':
					this.progs[prog] = spawn('node', [ process.env["UTCOUPE_WORKSPACE"] + '/lidar/main.js'/*, params.color, params.nb_erobots, params.EGR_d, params.EPR_d*/]);
				break;
			}

			logger.info("[Launch]"+prog);


			this.progs[prog].on('error', (err) => {
				this.server.to('webclient').emit('order', {
					to: 'webclient',
					name: 'logger',
					params: {
						head: '[ERROR]['+prog+'](code:'+err.code+')',
						text: convert.toHtml(JSON.stringify(err))
					},
					from: 'server'
				});
			});
			this.progs[prog].on('close', (code) => {
				logger.error("[CLOSE]"+prog);
				this.server.to('webclient').emit('order', {
					to: 'webclient',
					name: 'logger',
					// params: '[CLOSE]['+prog+'] '+data.toString(),
					params: {
						head: '[CLOSE]['+prog+'](code:'+code+')',
						text: " "
					},
					from: 'server'
				});
				this.kill(prog);
			});

			this.progs[prog].stdout.on('data', (data) => {
				this.server.to('webclient').emit('order', {
					to: 'webclient',
					name: 'logger',
					params: {
						head: '['+prog+'][stdout]',
						text: convert.toHtml(data.toString())
					},
					from: 'server'
				});
			});
			this.progs[prog].stderr.on('data', (data) => {
				this.server.to('webclient').emit('order', {
					to: 'webclient',
					name: 'logger',
					params: {
						source: '['+prog+'][stderr]',
						text: convert.toHtml(data.toString())
					},
					from: 'server'
				});
			});

				// logger.debug(prog);
				// logger.fatal(prog, '|stdout|', data.toString());
			this.utcoupe[prog] = true;
		}
		this.sendUTCoupe();
	}

	/**
	 * Kill the programm
	 * 
	 * @param {String} prog programm to kill
	 */
	kill (prog) {
		console.warn("server:kill not coded!");
		if(this.utcoupe[prog]) {
			// this.progs[prog].kill();
			// logger.info("stopped "+prog);
			this.server.to(prog).to("webclient").emit('order', {
                to: prog,
                name: 'kill',
                params: "",
                from: 'server'
            });
			this.utcoupe[prog] = false;
		}
		this.sendUTCoupe();
	 }

	/**
	 * sendUTCoupe
	 *
	 * @param {string} prog
	 */
	sendUTCoupe (prog) {
		this.server.to('webclient').emit('order', {
			to: 'webclient',
			name: 'utcoupe',
			params: this.utcoupe,
			from: 'server'
		});
	}

	/**
	 * sendUTCoupe
	 *
	 * @param {string} prog
	 */
	sendVerbosity () {
		this.server.to('webclient').emit('order', {
			to: 'webclient',
			name: 'serverVerbosity',
			params: {
				"isServerVerbose": this.verbose
			},
			from: 'server'
		});
	}
}

module.exports = Server;
