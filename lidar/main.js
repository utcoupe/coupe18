(function (){
	"use strict";

	var log4js = require('log4js');
	var logger = log4js.getLogger('lidar');
	var SocketClient = require('../server/socket_client.class.js');
	var server = require('../config.js').server;
	var Lidar = require('./lidar.class.js');

	var argv = process.argv;

	var client = new SocketClient({
		server_ip: server,
		type: "lidar",
	});

	var started = false;
	// var nb_active_hokuyos = -1;
	// var lastStatus = {
	// 	"status": "starting"
	// };
	// sendChildren(lastStatus);

	logger.info("Starting with pid " + process.pid);

	var hokMng = new Lidar(function (name, params){
		client.send("ia", name, params);
	}, function (newStatus, nbHok) {
		sendChildren({
			"status": newStatus,
			"nb": nbHok
		});
	});

	client.connect(function(){

		client.order(function(from, name, params){

			// var now = Date.now();
			// logger.info("Time since last order : "+(now - lastT));
			// if (now - lastT > 500) { // half a second between two orders
				//logger.info("Just received an order `" + name + "` from " + from + " with params :");
				//logger.info(params);

				// lastT = now;
				switch (name){
					case "start":
						if (!!params.color && !hokMng.started) {
							// logger.info("Receive order to start");
							hokMng.start(params.color);
						} else if (hokMng.started) {
							logger.error("Already started !");
						} else{
							logger.error("Missing parameters !");
						}
						break;
					case "hokuyo.polar_raw_data":
						if (hokMng.started) {
							hokMng.onHokuyoPolar(params.hokuyo, params.polarSpots);
						} else {
							// logger.warn("Start the Lidar before sending data !");
						}
						break;
					case "shutdown":
						// quitC("stop");
						spawn('sudo', ['halt']);
						break;
					case "calibration":
						hokMng.recalage(10); //Recalage des hokuyos
						break;
					case "color":
						hokMng.changeColor()
						break;
					case "kill":
						hokMng.stop();
						process.exit();
						// quitC("stop");
						break;
					case "sync_git":
						spawn('/root/sync_git.sh', [], {
							detached: true
						});
						break;
					default:
						logger.warn("Name " + name + " not understood : " + !!params?params:"");
				}
			// } else {
			// 	logger.warn("Received two orders too closely !");
			// }
		});
	});

	// function matchLogger(name, line){
	//	fs.appendFile('/var/log/utcoupe/'+name+'.log', line+'\n', function (err) {
	//		if (err) logger.error('Ecriture dans le fichier de log de match "/var/log/utcoupe/'+name+'.log" impossible');
	//		// logger.debug('The "data to append" was appended to file!');
	//	});
	// }

	// function getStatus(){
	//	var data = {
	//		"status": "",
	//		"children": []
	//	};

	//	switch (nb_active_hokuyos){
	//		case 0:
	//			data.status = "error";
	//			break;
	//		case 1:
	//			data.status = "ok";
	//			data.children =  ["Lonesome hokuyo"];
	//			break;
	//		case 2:
	//			data.status = "everythingIsAwesome";
	//			data.children =  ["Hokuyo 1", "Hokuyo 2"];
	//			break;
	//	}

	//	return data;
	// }


	// Sends status to server
	function sendChildren(status){
		client.send("server", "server.childrenUpdate", status);
		client.send("ia", "lidar.status", status);
		// client.send("ia", "hokuyo.nb_hokuyo", { nb: nb_active_hokuyos });
	}

	// function isOk(){
	//	if(lastStatus.status != "waiting")
	//		lastStatus = getStatus();

	//	client.send("ia", "isOkAnswer", lastStatus);
	//	client.send("server", "server.childrenUpdate", lastStatus);
	//	client.send("ia", "hokuyo.nb_hokuyo", { nb: nb_active_hokuyos });
	// }
})();
