angular.module('app').controller('IndexCtrl', ['$rootScope', '$scope', 'UTCoupe', 'Client',
	function($rootScope, $scope, UTCoupe, Client) {
	$rootScope.act_page = 'index';
	$scope.utcoupe = UTCoupe.utcoupe;
	$scope.network = UTCoupe.network;

	// Params
	if (!!$scope.network
		&& !!$scope.network.ia
		&& Object.keys($scope.network.ia).length > 0) {
		$scope.our_color = $scope.network.ia[Object.keys($scope.network.ia)[0]].color || 'blue';
		console.log($scope.network.ia[Object.keys($scope.network.ia)[0]].color);
		$scope.we_have_hats = $scope.network.ia[Object.keys($scope.network.ia)[0]].we_have_hats || false;
	} else {
		$scope.our_color = 'blue';
		// $scope.nb_erobots = 2;
		// $scope.EGR_d = 300;
		// $scope.EPR_d = 150;
		$scope.we_have_hats = false;
	}

	$scope.spawn = function(u) {
		Client.send('server', 'server.spawn', {
			prog: u,
			color: $scope.our_color,
			// nb_erobots: $scope.nb_erobots,
			// EGR_d: $scope.EGR_d,
			// EPR_d: $scope.EPR_d,
			we_have_hats: $scope.we_have_hats
		});
	}
	
	$scope.kill = function (u) {
		Client.send('server', 'server.kill', u);
	};
	$scope.start = function(u) {
		Client.send(u, 'start');
	};
	$scope.stop = function(u) {
		Client.send(u, 'stop');
	};
	$scope.place = function(u) {
		Client.send("ia", u + '.place');
	};

	$scope.jack = function() {
		Client.send('ia', 'ia.jack', {});
	}

	$scope.toogleVerbose = function() {
		Client.send('server', 'server.verbose', {});
	}

	$scope.startC = function() {
		Client.send("hokuyo", "start", {});
	}

	$scope.stopC = function() {
		Client.send("hokuyo", "stop", {});
	}

	$scope.calibration = function() {
		Client.send("hokuyo", "calibration", {});
	}

	$scope.resumeLidar = function() {
		Client.send("lidar", "start", {
			"color": $scope.our_color
		});
	}

	$scope.pauseLidar = function() {
		Client.send("lidar", "stop", {});
	}
	$scope.calibLidar = function() {
		Client.send("lidar", "calibration", {});
	}
}]);

angular.module('app').service('UTCoupe', ['$rootScope', 'Client', function($rootScope, Client) {
	this.utcoupe = {
		'ia': false,
		'pr': false,
		'gr': false,
		'lidar': false,
		'hokuyo': false,
		'isServerVerbose': false
	};
	this.network = {};
	this.init = function () {
		Client.order(function (from, name, data) {
			switch(name){
				case 'utcoupe':
					// console.log('[UTCoupe update]');
					this.utcoupe.ia = data.ia;
					this.utcoupe.gr = data.gr;
					this.utcoupe.pr = data.pr;
					this.utcoupe.lidar = data.lidar;
					this.utcoupe.hokuyo = data.hokuyo;
					if($rootScope.act_page == 'index') {
						$rootScope.$apply();
					}
					break;
				case 'reseau':
					this.network = data.network;
					break;
				case 'serverVerbosity':
					this.utcoupe.isServerVerbose = data.isServerVerbose;
					if($rootScope.act_page == 'index') {
						$rootScope.$apply();
					}
					break;
			}
		}.bind(this));
	};
}]);
