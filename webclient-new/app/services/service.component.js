class ServiceController {
  constructor($scope, $http, Ros) {
    this.$scope = $scope;
    this.$http = $http;
    this.ros = Ros;
  }

  $onInit() {
    const path = 'app/services/';
    this.fileName = `${path}default.html`;

    this.$scope.$watchGroup(['service.type', 'service.active'], () => {
      if(!this.service.active) {
        this.filename = `${path}disabled.html`;
        return;
      }
      if (!this.service.type) {
        this.fileName = `${path}default.html`;
        return;
      }

      const fileName = `${path}${this.service.type}.html`;
      this.$http.get(fileName).then((result) => {
        if (result.data) {
          this.fileName = fileName;
        }
      }, () => {});
    });
  }

  callService(input, isJSON) {
    // if(!this.service.active)
    //   return;

    const data = isJSON ? angular.fromJson(input) : input;
    const ROSservice = new ROSLIB.Service({
      ros: this.ros.ros,
      name: this.service.name,
      serviceType: this.service.type,
    });
    const request = new ROSLIB.ServiceRequest(data);

    ROSservice.callService(request, (result) => {
      this.result = result;
    });
  }
}

angular.module('roscc').component('ccService', {
  bindings: { service: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: ServiceController,
});
