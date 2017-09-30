angular.module('roscc').service('Hokuyo', ['$rootScope', '$sce', 'Ros',
function($rootScope, $sce, Ros) {
  this.displays = {
    main: null,
    one: null,
    two: null
  };

  this.lastData = {
    one: null,
    two: null,
    main: null
  };

  this.init = function () {
    //Client.order(this.onOrder); //FIXME
  };

  this.onOrder = function (from, name, data) {
    // if($rootScope.act_page == 'hokuyo') {
    if (name == 'hokuyo.polar_raw_data') {
      if (!!this.displays[data.hokuyo]) {
        // Save for later
        this.lastData[data.hokuyo] = data.polarSpots;

        // console.log("Received data from " + data.hokuyo);

        // Show
        if (!this.displays[data.hokuyo].isBusy) {
          this.displays[data.hokuyo].updatePolarSpots(data.polarSpots);
        }
      }
    } else if (name == 'lidar.all'
    && !!this.displays.main) {
      // Save for later
      this.lastData.main = {};
      this.lastData.main.hokuyos = data.hokuyos;
      this.lastData.main.robotsSpots = data.robotsSpots;
      this.lastData.main.cartesianSpots = data.cartesianSpots;

      // console.log("Received all");

      // Show
      if (!this.displays.main.isBusy) {
        this.displays.main.updateAll(data.hokuyos, data.robotsSpots, data.cartesianSpots);
      }
    } else if (name == 'lidar.light'
    && !!this.displays.main) {
      // Save for later
      this.lastData.main = {};
      this.lastData.main.hokuyos = data.hokuyos;
      this.lastData.main.robotsSpots = data.robotsSpots;

      // console.log("Received all");

      // Show
      if (!this.displays.main.isBusy) {
        this.displays.main.updateAll(data.hokuyos, data.robotsSpots, null);
      }
    } else if (name == 'lidar.robots'
    && !!this.displays.main) {
      // this.displays.main.updateRobots(data.robots);
    }
    // }
  }.bind(this);
}]);
