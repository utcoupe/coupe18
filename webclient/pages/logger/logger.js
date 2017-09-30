angular.module('app').controller('LoggerCtrl', ['$rootScope', '$scope', 'Logger',
	function($rootScope, $scope, Logger) {
	// $rootScope.act_page = 'logger';
	$scope.logs = Logger.logs;
}]);

angular.module('app').service('Logger', ['$rootScope', '$sce', 'Client',
	function($rootScope, $sce, Client) {
	this.logs = [];
	this.init = function () {
		var LOG_MAX_LENGTH = 500;
		Client.order(function (from, name, data) {
			if(name == 'logger') {
				// console.log('log', data);
				var head = data.head ? data.head : "";
				var text =  data.text ? data.text : "";
				var print = "<span class=\"logger_head\">"+head+"</head>"+text;
				this.logs.unshift($sce.trustAsHtml(print));
				this.logs.splice(LOG_MAX_LENGTH, 1000); //remove all logs index > 500
				
				if($rootScope.act_page == 'index') {
					$rootScope.$apply();
				}
			}
		}.bind(this));
	};
}]);
