// TODO :
// 		  message/objet erreur ou pas ?

(function (){
	"use strict";

	/* JS which will connect to the server, and then
	*  will execute the C program to control the Hokuyos.
	*  It will transfer datas from the C Hokuyo controller to AI
	*/

	var log4js = require('log4js');
	var logger = log4js.getLogger('Client');
	var spawn = require('child_process').spawn;
	var fs = require('fs');
	var match_name  = "";
	var child_process = require('child_process');
	var child;
	var SocketClient = require('../server/socket_client.class.js');
	var config = require('../config.js');
	var lastT = Date.now();
	var startingT = lastT;
	var bufferData;
	var count = 0;

	var FREQ_ENVOI_INFO = 50; // tous les 10 infos (genre 1 seconde)
	var nth = 0;

	var server = process.argv[2] || config.server;

	if (!process.env.UTCOUPE_WORKSPACE) {
		logger.warn("Missing UTCOUPE_WORKSPACE environment variable... Please make sur you have launched the install script");
		process.exit(1);
	}

	var client = new SocketClient({
		server_ip: server,
		type: "hokuyo"
	});

	var started = false;
	var nb_active_hokuyos = -1;
	var lastStatus = {
		"status": "waiting"
	};
	changeStatus("waiting");

	logger.info("Starting with pid " + process.pid);

	// Exit handlers
	//do something when app is closing
	// process.on('exit', exit); // doesn't seem to work, don't know why... https://nodejs.org/api/process.html#process_event_exit
	// catches ctrl+c event
	process.on('SIGINT', exit);
	// process.on('SIGTERM', quitC); // SIGTERM impossible to catch
	//catches uncaught exceptions
	//process.on('uncaughtException', uException);

	client.order(function(from, name, params){
		var now = Date.now();
		logger.info("Time since last order : "+(now - lastT));
		if (now - lastT > 500) { // half a second between two orders
			logger.info("Just received an order `" + name + "` from " + from + " with params :");
			logger.info(params);

			lastT = now;
			switch (name){
				case "start":
					if(!started) {
						started = true;
						logger.info("Receive order to start");
						start();
					} else
						logger.error("Already started !");
					break;
				case "shutdown":
					quitC("stop");
					spawn('sudo', ['halt']);
					break;
				case "stop":
					started = false;
					quitC("stop");
					break;
				/*
                                case "kill":
                                        started = false;
                                        quitC("stop");
                                        break;
				*/
				case "sync_git":
					spawn('/root/sync_git.sh', [], {
						detached: true
					});
					break;
				default:
					logger.warn("Name " + name + " not understood : " + params);
			}
		} else {
			logger.warn("Received two orders too closely !");
		}
	});

	function matchLogger(name, line){
		spawn("mkdir", ["-p", "/var/log/utcoupe"]);
		fs.appendFile('/var/log/utcoupe/'+name+'.log', line+'\n', function (err) {
			if (err) logger.error('Ecriture dans le fichier de log de match "/var/log/utcoupe/'+name+'.log" impossible');
			// logger.debug('The "data to append" was appended to file!');
		});
	}

	function exit(code) {
		logger.info("Closing Hokuyo C client");
		quitC(code);
		logger.info("Exiting");
		process.exit();
	}

	function quitC(code){
		if(!!child){
			logger.info("Closing child "+child.pid+" at "+code);
			child.kill('SIGINT');
			child = null;
		} else {
			logger.info("Can't close child at "+code+" : never born :P");
			logger.info("Father's pid : " + process.pid);
			// process.kill(process.pid, 'SIGINT');
		}
		started = false;
	}

	function uException(code){
		logger.error("uException sent with code "+code);
	}

	function start(){
		// We just an order to start, with the flavour :P (color, number of robots)

		changeStatus("starting");

		// Generates the match name (for the log file)
		var tmp = new Date();
		match_name = tmp.toJSON().replace(/T/, ' ').replace(/\..+/, '');
		var now = Date.now() - lastT;
		matchLogger(match_name, now);
		now = lastT;
		var logcount = 101;

		// If there's a child, kill it
		quitC("start");

		// Functions
		function parseData(string) {
			//logger.warn(string.length);
			var temp = string.split("#")
			if(temp[1] == "0" && count == 0){
				bufferData = temp[2];
				count = count + 1;
			}
			else {
				if(temp[1] == "1" && count == 1){
					bufferData = bufferData + temp[2];
					count = count + 1;
				}
				else {
					if(temp[1] == "2" && count == 2){
						bufferData = bufferData + temp[2];
						client.send("lidar", "hokuyo.polar_raw_data", { "hokuyo": temp[0], "polarSpots" : JSON.parse(bufferData) });
						count = 0;

						// logger.warn(JSON.parse(bufferData));
						if (logcount++ > 100) {
							logger.info("Hokuyo " + temp[0] + " correctly running");
							logcount = 0;
						}
					}
				}
			}
		}

		function parseInfo(string) {
			logger.info("Read info...");
			// logger.info(string);

			var prev_n_a_h = nb_active_hokuyos;

			now = Date.now() - lastT;

			switch (string.substring(0,1)){
				case "0":
					// Send error : no Hokuyo working
					// client.send("ia", "nb_hokuyo", {nb: 0});
					nb_active_hokuyos = 0;
					break;
				case "1":
					// Send warning : one Hokuyo is missing
					// client.send("ia", "nb_hokuyo", {nb: 1});
					nb_active_hokuyos = 1;
					break;
				case "2":
					// Send message : Hokuyos are ok
					// client.send("ia", "nb_hokuyo", {nb: 2});
					nb_active_hokuyos = 2;
					break;
				default:
					logger.info("Error not understood : " + string);
					return;
			}

			if ((prev_n_a_h != nb_active_hokuyos) || (nth == FREQ_ENVOI_INFO)){
				logger.info("Info sent to server");
				sendChildren(getStatus());
				nth = 0;
			}

			matchLogger(match_name, now+"; nb_hokuyo:"+nb_active_hokuyos);
			now = lastT;
			nth += 1;
		}

		function dataFromCHandler(input) {
			// input format (XXXX type and xxxx values) : "[XXXX]xxxxxxxxx" maybe many times, seperated with \n

			var inputAr = input.toString().split('\n');

			for (var i = 0; i <= inputAr.length - 1; i++) {
				if (!!inputAr[i]){
					switch (inputAr[i].substring(1,5)){
						case "HI:)":
							// send "C started" to server
							logger.info('C Hokuyo software says "Hi !" :)');
							changeStatus("starting");
							break;
						case "DATA":
							//logger.info('C Hokuyo software sends datas');
							parseData(inputAr[i].substring(6));
							changeStatus("everythingIsAwesome");
							break;
						case "INFO":
							logger.info('C Hokuyo software sends information :'+inputAr[i].substring(6));
							//parseInfo(inputAr[i].substring(6));
							break;
						case "WARN":
							logger.warn('C Hokuyo software sends a warning :'+inputAr[i].substring(6));
							//parseInfo(inputAr[i].substring(6));
							break;
						default:
							logger.info("Data "+ inputAr[i].substring(1,5) + " not understood at line " + i + " : " + inputAr[i]);
					}
				}
			}
		}

		// Execute C program
		var command = process.env.UTCOUPE_WORKSPACE + "/bin/hokuyo";
		var args = []; // [color];
		// var options = // default : { cwd: undefined, env: process.env};
		logger.info('Launching : ' + command + ' ' + args);
		child = child_process.spawn(command, args);
		logger.info("process C lance");

		// Events
		child.stdout.on('data', function(data) {
			//logger.debug(data.toString());
			dataFromCHandler(data);
		});

		child.on('error', function(data) {
			logger.fatal('Erreur avec le process C : ' + data.toString());
			sendChildren({"status": "error", "children":[]});
			setTimeout(function(){
				sendChildren({"status": "waiting", "children":[]});
			}, 5000);
		});

		child.stderr.on('error', function(data) {
			logger.fatal(data.toString());
			sendChildren({"status": "error", "children":[]});
			setTimeout(function(){
				sendChildren({"status": "waiting", "children":[]});
			}, 5000);
		});

		child.stderr.on('data', function(data) {
			logger.error(data.toString());
			sendChildren({"status": "error"});
			setTimeout(function(){
				sendChildren({"status": "waiting", "children":[]});
			}, 5000);
		});

		child.on('close', function(code) {
			started = false;
			if (code == 0)
				logger.info('Child closed correctly');
			else
				logger.error('Child closed with code: ' + code);

			// Send message
			if (code != -1)
				sendChildren({"status": "waiting", "children":[]});
		});
	}

	function getStatus(){
		var data = {
			"status": "",
			"children": []
		};

		switch (nb_active_hokuyos){
			case 0:
				data.status = "error";
				break;
			case 1:
				data.status = "ok";
				data.children =  ["Lonesome hokuyo"];
				break;
			case 2:
				data.status = "everythingIsAwesome";
				data.children =  ["Hokuyo 1", "Hokuyo 2"];
				break;
		}

		return data;
	}

	function changeStatus(newStatus) {
		if (newStatus != lastStatus.status) {
			logger.info("New status : " + newStatus);
			lastStatus.status = newStatus;
			sendChildren(lastStatus);
		}
	}

	// Sends status to server
	function sendChildren(status){
		lastStatus = status;

		client.send("server", "server.childrenUpdate", lastStatus);
	}

	function isOk(){
		if(lastStatus.status != "waiting")
			lastStatus = getStatus();

		client.send("ia", "isOkAnswer", lastStatus);
		client.send("server", "server.childrenUpdate", lastStatus);
	}
})();
