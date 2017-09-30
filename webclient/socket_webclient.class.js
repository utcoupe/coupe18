SocketWebclient = (function () {
	"use strict";

	function SocketWebclient() {
		this.server_host = window.location.hostname+':3128';
		this.socket = null;
		this.callbacks = {};
		this.type = this.getDeviceType();

		if(io === undefined) {
			this.errorSocketIoNotFound();
		} else {
			this.socket = io(this.server_host);
			
			this.socket.on('connect', function(){
				this.socket.emit('type', {
					type: 'webclient',
					options: {
						name: 'Webclient',
						type: this.type
					}
				});
				if(!!this.callbacks.connect)
					this.callbacks.connect();
				// this.socket.emit('order', {to:'webclient',text:'Hello!'});
			}.bind(this));

			// When the client is disconnected from the server
			this.socket.on('disconnect', function(){
				this.errorServerTimeout();
				console.log('Client disconnected from server');
				for(var i in this.callbacks.orders) {
					this.callbacks.orders[i]("socket_client.class.js", "reseau", {network: null});
				}
			}.bind(this));

			this.socket.on('log', function(data){
				console.log('[Server log] '+data);
			});
			this.socket.on('order', function(data){
				// console.log('[Order to '+data.to+'] '+ data.text);
				if(!!this.callbacks.orders) {
					if (!!data.name){
						// console.log("Order " + data.name + " from " + data.from + "with params :");
						// console.log(data.params);
						for(var i in this.callbacks.orders) {
							this.callbacks.orders[i](data.from, data.name, data.params || {}, data.to);
						}
					}
					else
						console.log("Order has no name ! : " + data);
				}
			}.bind(this));

			setTimeout(function() {
				if(this.socket.disconnected)
					this.errorServerNotFound();
			}.bind(this), 500);
		}
	}

	SocketWebclient.prototype.getDeviceType = function () {
		if (navigator.userAgent.match(/(android|iphone|ipad|blackberry|symbian|symbianos|symbos|netfront|model-orange|javaplatform|iemobile|windows phone|samsung|htc|opera mobile|opera mobi|opera mini|presto|huawei|blazer|bolt|doris|fennec|gobrowser|iris|maemo browser|mib|cldc|minimo|semc-browser|skyfire|teashark|teleca|uzard|uzardweb|meego|nokia|bb10|playbook)/gi)) {
			if ( ((screen.width  >= 480) && (screen.height >= 800)) || ((screen.width  >= 800) && (screen.height >= 480)) || navigator.userAgent.match(/ipad/gi) ) {
				return 'tablet';
			}
			else {
				return 'smartphone'; 
			}
		}
		else {
			return 'laptop';
		}
	};

	SocketWebclient.prototype.connect = function (callback) {
		this.callbacks.connect = callback;
	};

	SocketWebclient.prototype.order = function (callback) {
		if(!this.callbacks.orders)
			this.callbacks.orders = [];
		this.callbacks.orders.push(callback);
	};

	SocketWebclient.prototype.send = function (to, name, params) {
		this.socket.emit('order', {
			to: to,
			name: name,
			params: params,
			from: this.type
		});
	};

	// Error functions
	SocketWebclient.prototype.error = function (msg, reload) {
		$('#webclient').html('<p class="error" ><strong>Error</strong>: '+msg+'</p>');
	};
	SocketWebclient.prototype.errorServerNotFound = function () {
		this.error('server not found at '+this.server_host+'.<br />Please make sure the server is running.', false);
	};
	SocketWebclient.prototype.errorServerTimeout = function () {
		this.error('server timed out.<br />Please make sure the server is still running.', false);
	};
	SocketWebclient.prototype.errorSocketIoNotFound = function () {
		this.error('socket.io.js not found.<br />Please make sure you are in webclient/ folder.', false);
	};

	return SocketWebclient;
})();
