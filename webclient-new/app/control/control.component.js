
class ControlController {
  constructor($rootScope, $timeout, $interval, Settings, Domains, Ros) {
    this.$timeout = $timeout;
    this.Domains = Domains;

    this.ros = Ros;
    this.setting = Settings.get();
    this.maxConsoleEntries = 200;

    this.resetData();
    if($rootScope.isConnected) {
      this.$timeout(() => { this.onConnected(); }, 500);
    } else {
      $rootScope.$watch('isConnected', function(newValue, oldValue) {
        if(newValue)
          this.$timeout(() => { this.onConnected(); }, 500);
      }.bind(this));
    }

  }

  // The active domain shows further information in the center view
  setActiveDomain(domain) {
    this.activeDomain = domain;
  }

  getDomains() {
    const allData = this.data.topics.concat(this.data.services, this.data.nodes);
    const domains = this.Domains.getDomains(allData);

    if (!this.activeDomain) {
      this.setActiveDomain(domains[0]);
    }
    return domains;
  }

  hasFilteredDomains(advanced) {
    return _.some(_.map(this.getDomains(), dom => this.Domains.filterAdvanced(dom, advanced)));
  }

  getGlobalParameters() {
    return this.Domains.getGlobalParameters(this.data.parameters);
  }

  resetData() {
    this.data = {
      rosout: [],
      topics: [],
      nodes: [],
      parameters: [],
      services: [],
    };
  }

  onConnected() {
    this.loadData();

    this.setConsole();
    if (this.setting.battery) {
      this.setBattery();
    }
  }

  // Setup of console (in the right sidebar)
  setConsole() {
    const consoleTopic = new ROSLIB.Topic({
      ros: this.ros.ros,
      name: this.setting.log,
      messageType: 'rosgraph_msgs/Log',
    });
    consoleTopic.subscribe((message) => {
      const nameArray = message.name.split('/');
      const d = new Date((message.header.stamp.secs * 1E3) + (message.header.stamp.nsecs * 1E-6));

      message.abbr = (nameArray.length > 1) ? nameArray[1] : message.name;

      // String formatting of message time and date
      function addZero(i) { return i < 10 ? `0${i}` : `${i}`; }
      message.dateString = `${addZero(d.getHours())}:
      ${addZero(d.getMinutes())}:
      ${addZero(d.getSeconds())}.
      ${addZero(d.getMilliseconds())}`;
      this.data.rosout.unshift(message);

      if (this.data.rosout.length > this.maxConsoleEntries) {
        this.data.rosout.pop();
      }
    });
  }

  // Setup battery status
  setBattery() {
    const batteryTopic = new ROSLIB.Topic({
      ros: this.ros.ros,
      name: this.setting.batteryTopic,
      messageType: 'std_msgs/Float32',
    });
    batteryTopic.subscribe((message) => {
      this.batteryStatus = message.data;
    });
  }

  // Load structure, all data, parameters, topics, services, nodes...
  loadData() {
    this.resetData();

    this.ros.ros.getTopics((topics) => { // Topics now has topics and types arrays
      angular.forEach(topics.topics, (name) => {
        this.data.topics.push({ name });

        this.ros.ros.getTopicType(name, (type) => {
          _.findWhere(this.data.topics, { name }).type = type;
        });
      });
    });

    this.ros.ros.getServices((services) => {
      angular.forEach(services, (name) => {
        this.data.services.push({ name });

        this.ros.ros.getServiceType(name, (type) => {
          _.findWhere(this.data.services, { name }).type = type;
        });
      });
    });

    this.ros.ros.getParams((params) => {
      angular.forEach(params, (name) => {
        const param = new ROSLIB.Param({ ros: this.ros.ros, name });
        this.data.parameters.push({ name });

        param.get((value) => {
          _.findWhere(this.data.parameters, { name }).value = value;
        });
      });
    });

    this.ros.ros.getNodes((nodes) => {
      angular.forEach(nodes, (name) => {
        this.data.nodes.push({ name });
      });
    });
  }
}

angular.module('roscc').component('ccControl', {
  templateUrl: 'app/control/control.html',
  controller: ControlController,
});
