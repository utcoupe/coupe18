angular.module('app').controller('TelecommandeCtrl', ['$rootScope', '$scope', 'Client',
	function($rootScope, $scope, Client) {
	$rootScope.act_page = 'telecommande';

	// ********************************* GIT *********************************
	$scope.prSyncGit = function() {
		Client.send("pr", "sync_git");
	}

	$scope.hokSyncGit = function() {
		Client.send("hokuyo", "sync_git");
	}

	$scope.serverSyncAllGit = function() {
		Client.send("server", "server.sync_all_git");
	}

	$scope.sendFinished = function(){
		Client.send("ia", $("#robot_finished").val() + $("#what_finished").val());
	}

	$scope.serverFlashArduinos = function() {
		Client.send("server", "server.flash_arduinos");
	}

	// ******************************** Match ********************************
	$scope.iaJack = function() {
		Client.send("ia", "ia.jack");
	}

	$scope.iaStop = function() {
		Client.send("ia", "ia.stop");
	}

	// ******************************** Tibot ********************************
	if (!$scope.tibot)
		$scope.tibot = new TibotDisplay("pr", Client);

	// ******************************** Grobot ********************************
	if (!$scope.grobot)
		$scope.grobot = new GrobotDisplay("gr", Client);
}]);
