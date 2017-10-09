
class DiagnosticController {
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
      $rootScope.$watch('isConnected', function(newValue) {
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
    this.setConsole();

    if (!this.activeDomain) {
      for(let d of this.domains) {
        if(this.ros.getDomains().includes(d.name)) {
          this.setActiveDomain(d.name);
        }
      }
    }
  }

  // Setup of console (in the right sidebar)
  setConsole() {
    this.consoleTopic = new ROSLIB.Topic({
      ros: this.ros.ros,
      name: this.setting.log,
      messageType: 'rosgraph_msgs/Log',
    });
    this.consoleTopic.subscribe((message) => {
      const nameArray = message.name.split('/');
      const d = new Date((message.header.stamp.secs * 1E3) + (message.header.stamp.nsecs * 1E-6));

      message.abbr = (nameArray.length > 1) ? nameArray[1] : message.name;

      // String formatting of message time and date
      function addZero(i) { return i < 10 ? `0${i}` : `${i}`; }
      message.dateString = `${addZero(d.getHours())}:
      ${addZero(d.getMinutes())}:
      ${addZero(d.getSeconds())}.
      ${addZero(d.getMilliseconds())}`;

      this.ros.data.rosout.unshift(message);

      if (this.ros.data.rosout.length > this.maxConsoleEntries) {
        this.ros.data.rosout.pop();
      }
    });
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

angular.module('roscc').component('ccDiagnostic', {
  templateUrl: 'app/diagnostic/diagnostic.html',
  controller: DiagnosticController,
});
