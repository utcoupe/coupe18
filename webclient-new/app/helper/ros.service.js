class RosService {
  constructor($rootScope, $log, $interval, $timeout, Settings, Domains) {
    $rootScope.isConnected = false;
    this.isConnected = $rootScope.isConnected;

    this.setting = Settings.get();
    this.$interval = $interval;
    this.$timeout = $timeout;
    this.$log = $log;
    this.$rootScope = $rootScope;
    this.ros = false;
    this.Domains = Domains;

    this.resetData();
    this.newRosConnection();
    $interval(() => {
      this.newRosConnection();
    }, 1000); // [ms]
  }

  newRosConnection(callback) {
    if (this.$rootScope.isConnected || !this.setting) {
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
      this.loadData();
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
      this.$timeout(function() {
        callback();
      }.bind(this), 1000); // [ms]
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
    return topic;
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



  resetData() {
    this.data = {
      rosout: [],
      topics: [],
      nodes: [],
      parameters: [],
      services: []
    };
  }

  // Load structure, all data, parameters, topics, services, nodes...
  loadData() {
    this.resetData();

    this.ros.getTopics((topics) => { // Topics now has topics and types arrays
      angular.forEach(topics.topics, (name) => {
        let t = {
          name: name,
          active: true
        }
        this.data.topics.push(t);
        this.ros.getTopicType(name, (type) => {
          _.findWhere(this.data.topics, t).type = type;
        });
      });

      for(let d of this.$rootScope.domains) {
        for(let t of d.topics) {
          let name = '/'+d.name+'/'+t;
          if(!_.some(this.data.topics, (active) => active.name == name )) {
            let newT = {
              name: name,
              abbr: t,
              active: false
            };
            this.data.topics.push(newT);
          }
        }
      }
    });

    this.ros.getServices((services) => {
      angular.forEach(services, (name) => {
        let s = {
          name: name,
          active: true
        }
        this.data.services.push(s);
        this.ros.getServiceType(name, (type) => {
          _.findWhere(this.data.services, s).type = type;
        });
      });


      for(let d of this.$rootScope.domains) {
        for(let s of d.services) {
          let name = '/'+d.name+'/'+s;
          if(!_.some(this.data.services, (active) => active.name == name )) {
            let newS = {
              name: name,
              abbr: s,
              active: false
            };
            this.data.services.push(newS);
          }
        }
      }
    });

    this.ros.getParams((params) => {
      angular.forEach(params, (name) => {
        const param = new ROSLIB.Param({ ros: this.ros, name });
        this.data.parameters.push({ name });

        param.get((value) => {
          _.findWhere(this.data.parameters, { name }).value = value;
        });
      });
    });

    this.ros.getNodes((nodes) => {
      angular.forEach(nodes, (name) => {
        this.data.nodes.push({ name });
      });
    });
  }


  getDomains() {
    if(!this.data)
      return;
    const allData = this.data.topics.concat(this.data.services, this.data.nodes);
    const domains = this.Domains.getDomains(allData);

    let expectedD = _.pluck(this.$rootScope.domains, 'name');

    //set expected domains first in the list
    return _.uniq(expectedD.concat(domains));
  }

  getExpectedDomains() {
    return _.pluck(this.$rootScope.domains, 'name');
  }

  getUnexpectedDomains() {
    return _.difference(this.getDomains(), this.getExpectedDomains());
  }

  getTopicsForDomain(domain) {
    return this.Domains.getDataForDomain(this.data.topics, domain, false);
  }

  getServicesForDomain(domain) {
    return this.Domains.getDataForDomain(this.data.services, domain, false);
  }

  getGlobalParameters() {
    return this.Domains.getGlobalParameters(this.data.parameters);
  }



}

angular.module('roscc').service('Ros', RosService);
