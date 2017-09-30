class RosService {
  constructor($rootScope, $log, $interval, $timeout, Settings) {
    $rootScope.isConnected = false;
    this.isConnected = $rootScope.isConnected;

    this.setting = Settings.get();
    this.$interval = $interval;
    this.$timeout = $timeout;
    this.$log = $log;
    this.$rootScope = $rootScope;
    this.ros = false;

    this.newRosConnection();
    $interval(() => {
      this.newRosConnection();
    }, 1000); // [ms]
  }

  newRosConnection(callback) {
    if (this.$rootScope.isConnected || this.setting === angular.isUndefined) {
      if(callback) callback();
      return;
    }

    if (this.ros) {
      this.ros.close(); // Close old connection
      this.ros = false;
      return;
    }

    this.ros = new ROSLIB.Ros({ url: `ws://${this.setting.address}:${this.setting.port}` });

    this.ros.on('connection', () => {
      this.$rootScope.isConnected = true;
      this.isConnected = this.$rootScope.isConnected;
      this.$log.log('Successfully connected to server !');
    });

    this.ros.on('error', () => {
      this.$rootScope.isConnected = false;
      this.isConnected = this.$rootScope.isConnected;
      this.ros = false;
      this.$log.log('Error trying to connect to server !');

    });

    this.ros.on('close', () => {
      this.$rootScope.isConnected = false;
      this.isConnected = this.$rootScope.isConnected;
      this.$log.log('Connection to server closed !');
    });


    if(callback) {
      this.$timeout(() => {
        callback();
      }, 1000); // [ms]
    }
  }


  sendOrder(to, data, callback) {
    if(!this.ros) return;
    let service = new ROSLIB.Service({ ros: this.ros, name: to });
    let request = new ROSLIB.ServiceRequest(data);
    service.callService(request, callback)
  }

  publish(to, data) {
    if(!this.ros) return;
    let topic = new ROSLIB.Topic({ ros: this.ros, name: to })
    let msg = new ROSLIB.Message(data);
    topic.publish(msg);
  }

  listen(from, callback) {
    if(!this.ros) return;
    let topic = new ROSLIB.Topic({ ros: this.ros, name: from })
    topic.subscribe(callback);
  }

  getParam(name, callback) {
    if(!this.ros) return;
    let param = new ROSLIB.Param({ ros: this.ros, name: name })
    param.get(callback);
  }

  setParam(name, value) {
    if(!this.ros) return;
    let param = new ROSLIB.Param({ ros: this.ros, name: name })
    param.set(value);
  }


}

angular.module('roscc').service('Ros', RosService);
