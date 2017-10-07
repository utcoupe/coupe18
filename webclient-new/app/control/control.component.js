
class ControlController {
  constructor($rootScope, $timeout, $interval, Settings, Domains, Ros) {
    this.$timeout = $timeout;
    this.Domains = Domains;
    this.domains = $rootScope.domains;

    this.ros = Ros;
    this.setting = Settings.get();
    this.maxConsoleEntries = 200;

    if($rootScope.isConnected) {
      this.$timeout(() => { this.onConnected(); }, 1000);
    } else {
      $rootScope.$watch('isConnected', function(newValue, oldValue) {
        if(newValue)
          this.$timeout(() => { this.onConnected(); }, 1000);
      }.bind(this));
    }

  }

  // The active domain shows further information in the center view
  setActiveDomain(domain) {
    this.activeDomain = domain;
  }


  onConnected() {

    if (!this.activeDomain) {
      for(let d of this.domains) {
        if(this.ros.getDomains().includes(d.name)) {
          this.setActiveDomain(d.name);
        }
      }
    }
  }



  refresh() {
    this.ros.loadData();
  }

  isDomainActive(domain) {
    return _.some(this.ros.getTopicsForDomain(domain), (t) => t.active == true);
  }

  $onDestroy() {
    this.consoleTopic.unsubscribe();
  }
}

angular.module('roscc').component('ccControl', {
  templateUrl: 'app/control/control.html',
  controller: ControlController,
});
