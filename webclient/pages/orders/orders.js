angular.module('app').controller('OrdersCtrl', ['$rootScope', '$scope', 'Orders',
	function($rootScope, $scope, Orders) {
	$rootScope.act_page = 'orders';
	$scope.orders = Orders.orders;
}]);

angular.module('app').service('Orders', ['$rootScope', 'Client', function($rootScope, Client) {
	this.orders = [];
	this.init = function () {
		Client.order(function (from, name, data, to) {
			// Be careful, some orders are filtered by the server, set verbose mode to see them or go to server.server.class.js:175

			if (name != "logger"
				&& name != "serverVerbosity"
				&& name != "utcoupe"
				&& name != "reseau"
				&& name != "simulateur") {
				this.orders.unshift({
					from: from,
					name: name,
					data: JSON.stringify(data),
					to: to,
				});
				if(this.orders.length > 500)
					this.orders.pop();
				if($rootScope.act_page == 'orders') {
					$rootScope.$apply();
				}
			}
		}.bind(this));
	};
}]);
