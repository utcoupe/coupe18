'use strict';

function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider.when('/reseau', { template: '<cc-reseau></cc-reseau>' }).when('/diagnostic', { template: '<cc-diagnostic></cc-diagnostic>' }).when('/asserv', { template: '<cc-asserv></cc-asserv>' }).when('/simulateur', { template: '<cc-simulateur></cc-simulateur>' }).when('/hokuyo', { template: '<cc-hokuyo></cc-hokuyo>' }).when('/telecommande', { template: '<cc-control></cc-control>' }).when('/settings', { template: '<cc-settings></cc-settings>' }).otherwise({ redirectTo: '/diagnostic' });

  localStorageServiceProvider.setPrefix('roscc');
}

function run($rootScope) {

  $rootScope.domains = [{
    name: 'drivers',
    topics: ['ard_asserv/pose2d', 'ard_asserv/speed'],
    services: ['ard_asserv/emergency_stop', 'ard_asserv/goto', 'ard_asserv/set_pos', 'ard_asserv/speed', 'ard_asserv/pwm', 'ard_asserv/management', 'ard_asserv/parameters']
  }, {
    name: 'ai',
    topics: ['oui1', 'non1'],
    services: ['s1', 's2']
  }, {
    name: "recognition",
    topics: ['oui1', 'non1'],
    services: ['gzfd']
  }, {
    name: "processing",
    topics: ['oui1', 'non1'],
    services: ['etr']
  }, {
    name: "sensors",
    topics: ['oui1', 'non1'],
    services: ['trezz']
  }, {
    name: "movement",
    topics: ['oui1', 'non1'],
    services: []
  }, {
    name: "memory",
    topics: ['oui1', 'non1'],
    services: []
  }, {
    name: "feedback",
    topics: ['oui1', 'non1'],
    services: []
  }, {
    name: "navigation",
    topics: ['oui1', 'non1'],
    services: []
  }];
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule', 'chart.js', 'ngAnimate']).config(ROSCCConfig).run(run);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var AsservController = function () {
  function AsservController(Ros) {
    _classCallCheck(this, AsservController);

    this.ros = Ros.ros;

    Ros.listen('/drivers/ard_asserv/pose2d', function (e) {
      this.pushDataToChart(2, e.x);
      this.pushDataToChart(3, e.theta);
      this.pushDataToChart(4, e.y);
    }.bind(this));

    Ros.listen('/drivers/ard_asserv/speed', function (e) {
      this.pushDataToChart(0, e.pwm_speed_left);
      this.pushDataToChart(1, e.pwm_speed_right);
      this.pushDataToChart(5, e.wheel_speed_right);
      this.pushDataToChart(7, e.wheel_speed_right);
      this.pushDataToChart(6, e.linear_speed);
    }.bind(this));

    // topics to listen to
    this.topics = ['/asserv/x', '/asserv/x', '/asserv/x', '/asserv/x', '/asserv/x', '/asserv/x', '/asserv/x', '/asserv/x'];

    this.options = {
      scales: {
        xAxes: [{
          ticks: {
            display: false
          },
          gridLines: {
            display: false
          }
        }]
      },
      animation: false,
      title: {
        display: true,
        text: 'Custom Chart Title',
        fontSize: 20
      }
    };

    this.datasetOverride = {
      label: 'Sinus',
      borderColor: 'rgb(75, 192, 192)',
      borderWidth: 2,
      pointRadius: 0,
      fill: false
    };

    this.charts = [];

    for (var i = 0; i < 8; i++) {

      this.charts.push({
        data: [0],
        labels: [0],
        options: JSON.parse(JSON.stringify(this.options)),
        datasetOverride: this.datasetOverride
      });
    }

    this.charts[0].options.title.text = 'PWM speed left';
    this.charts[1].options.title.text = 'PWM speed right';
    this.charts[2].options.title.text = 'X position';
    this.charts[3].options.title.text = 'Orientation';
    this.charts[4].options.title.text = 'Y position';
    this.charts[5].options.title.text = 'Wheel speed left';
    this.charts[7].options.title.text = 'Wheel speed right';
    this.charts[6].options.title.text = 'Linear speed';

    var canvas = document.getElementsByTagName('canvas');
    var _iteratorNormalCompletion = true;
    var _didIteratorError = false;
    var _iteratorError = undefined;

    try {
      for (var _iterator = canvas[Symbol.iterator](), _step; !(_iteratorNormalCompletion = (_step = _iterator.next()).done); _iteratorNormalCompletion = true) {
        var c = _step.value;
        fitToContainer(c);
      }
    } catch (err) {
      _didIteratorError = true;
      _iteratorError = err;
    } finally {
      try {
        if (!_iteratorNormalCompletion && _iterator.return) {
          _iterator.return();
        }
      } finally {
        if (_didIteratorError) {
          throw _iteratorError;
        }
      }
    }

    function fitToContainer(canvas) {
      // Make it visually fill the positioned parent
      canvas.style.width = '100%';
      canvas.style.height = '100%';
      // ...then set the internal size to match
      canvas.width = canvas.offsetWidth;
      canvas.height = canvas.offsetHeight;
    }
  }

  _createClass(AsservController, [{
    key: 'pushDataToChart',
    value: function pushDataToChart(i, e) {
      this.charts[i].data.push(e);
      this.charts[i].labels.push(this.charts[i].labels[this.charts[i].labels.length - 1] + 0.1);
    }
  }, {
    key: '$onInit',
    value: function $onInit() {
      var canvas = document.querySelector('canvas');
      fitToContainer(canvas);

      function fitToContainer(canvas) {
        // Make it visually fill the positioned parent
        canvas.style.width = '100%';
        canvas.style.height = '100%';
        // ...then set the internal size to match
        canvas.width = canvas.offsetWidth;
        canvas.height = canvas.offsetHeight;
      }
    }
  }]);

  return AsservController;
}();

angular.module('roscc').component('ccAsserv', {
  templateUrl: 'app/asserv/asserv.html',
  controller: AsservController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var ControlController = function () {
  function ControlController($rootScope, $timeout, $interval, Settings, Domains, Ros, Console) {
    var _this = this;

    _classCallCheck(this, ControlController);

    this.$timeout = $timeout;
    this.Domains = Domains;
    this.domains = $rootScope.domains;
    this.logs = Console.logs;

    this.ros = Ros;
    this.setting = Settings.get();

    if ($rootScope.isConnected) {
      this.$timeout(function () {
        _this.onConnected();
      }, 100);
    } else {
      $rootScope.$watch('isConnected', function (newValue) {
        var _this2 = this;

        if (newValue) this.$timeout(function () {
          _this2.onConnected();
        }, 100);
      }.bind(this));
    }
  }

  // The active domain shows further information in the center view


  _createClass(ControlController, [{
    key: 'setActiveDomain',
    value: function setActiveDomain(domain) {
      this.activeDomain = domain;
    }
  }, {
    key: 'onConnected',
    value: function onConnected() {

      if (!this.activeDomain) {
        var _iteratorNormalCompletion = true;
        var _didIteratorError = false;
        var _iteratorError = undefined;

        try {
          for (var _iterator = this.domains[Symbol.iterator](), _step; !(_iteratorNormalCompletion = (_step = _iterator.next()).done); _iteratorNormalCompletion = true) {
            var d = _step.value;

            if (this.ros.getDomains().includes(d.name)) {
              this.setActiveDomain(d.name);
              return;
            }
          }
        } catch (err) {
          _didIteratorError = true;
          _iteratorError = err;
        } finally {
          try {
            if (!_iteratorNormalCompletion && _iterator.return) {
              _iterator.return();
            }
          } finally {
            if (_didIteratorError) {
              throw _iteratorError;
            }
          }
        }
      }
    }
  }, {
    key: 'refresh',
    value: function refresh() {
      this.ros.loadData();
    }
  }, {
    key: 'isDomainActive',
    value: function isDomainActive(domain) {
      return _.some(this.ros.getServicesForDomain(domain), function (t) {
        return t.active == true;
      });
    }
  }, {
    key: 'collapseAll',
    value: function collapseAll(domain) {
      this.ros.getServicesForDomain(domain).map(function (item) {
        item.isOpen = false;
      });
    }
  }, {
    key: 'expandAll',
    value: function expandAll(domain) {
      this.ros.getServicesForDomain(domain).map(function (item) {
        item.isOpen = true;
      });
    }
  }]);

  return ControlController;
}();

angular.module('roscc').component('ccControl', {
  templateUrl: 'app/control/control.html',
  controller: ControlController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var DiagnosticController = function () {
  function DiagnosticController($rootScope, $timeout, $interval, Settings, Domains, Ros, Console) {
    var _this = this;

    _classCallCheck(this, DiagnosticController);

    this.$timeout = $timeout;
    this.Domains = Domains;
    this.domains = $rootScope.domains;
    this.logs = Console.logs;

    this.ros = Ros;
    this.setting = Settings.get();

    if ($rootScope.isConnected) {
      this.$timeout(function () {
        _this.onConnected();
      }, 1000);
    } else {
      $rootScope.$watch('isConnected', function (newValue) {
        var _this2 = this;

        if (newValue) this.$timeout(function () {
          _this2.onConnected();
        }, 1000);
      }.bind(this));
    }
  }

  // The active domain shows further information in the center view


  _createClass(DiagnosticController, [{
    key: 'setActiveDomain',
    value: function setActiveDomain(domain) {
      this.activeDomain = domain;
    }
  }, {
    key: 'onConnected',
    value: function onConnected() {
      if (!this.activeDomain) {
        var _iteratorNormalCompletion = true;
        var _didIteratorError = false;
        var _iteratorError = undefined;

        try {
          for (var _iterator = this.domains[Symbol.iterator](), _step; !(_iteratorNormalCompletion = (_step = _iterator.next()).done); _iteratorNormalCompletion = true) {
            var d = _step.value;

            if (this.ros.getDomains().includes(d.name)) {
              this.setActiveDomain(d.name);
            }
          }
        } catch (err) {
          _didIteratorError = true;
          _iteratorError = err;
        } finally {
          try {
            if (!_iteratorNormalCompletion && _iterator.return) {
              _iterator.return();
            }
          } finally {
            if (_didIteratorError) {
              throw _iteratorError;
            }
          }
        }
      }
    }
  }, {
    key: 'refresh',
    value: function refresh() {
      this.ros.loadData();
    }
  }, {
    key: 'isDomainActive',
    value: function isDomainActive(domain) {
      return _.some(this.ros.getTopicsForDomain(domain), function (t) {
        return t.active == true;
      });
    }
  }, {
    key: 'collapseAll',
    value: function collapseAll(domain) {
      this.ros.getTopicsForDomain(domain).map(function (item) {
        item.isOpen = false;
      });
    }
  }, {
    key: 'expandAll',
    value: function expandAll(domain) {
      this.ros.getTopicsForDomain(domain).map(function (item) {
        item.isOpen = true;
      });
    }
  }]);

  return DiagnosticController;
}();

angular.module('roscc').component('ccDiagnostic', {
  templateUrl: 'app/diagnostic/diagnostic.html',
  controller: DiagnosticController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var ConsoleService = function () {
  function ConsoleService(Ros, Settings, $rootScope) {
    var _this = this;

    _classCallCheck(this, ConsoleService);

    this.ros = Ros;
    this.setting = Settings.get();

    this.logs = [];
    $rootScope.$watch('isConnected', function () {
      if ($rootScope.isConnected) _this.setConsole();else if (_this.consoleTopic) _this.consoleTopic.unsubscribe();
    });
  }

  // Setup of console (in the right sidebar)


  _createClass(ConsoleService, [{
    key: 'setConsole',
    value: function setConsole() {
      var _this2 = this;

      this.consoleTopic = new ROSLIB.Topic({
        ros: this.ros.ros,
        name: this.setting.log,
        messageType: 'rosgraph_msgs/Log'
      });
      this.consoleTopic.subscribe(function (message) {
        var nameArray = message.name.split('/');
        var d = new Date(message.header.stamp.secs * 1E3 + message.header.stamp.nsecs * 1E-6);

        message.abbr = nameArray.length > 1 ? nameArray[1] : message.name;

        // String formatting of message time and date
        function addZero(i) {
          return i < 10 ? '0' + i : '' + i;
        }
        message.dateString = addZero(d.getHours()) + ':\n      ' + addZero(d.getMinutes()) + ':\n      ' + addZero(d.getSeconds());

        _this2.logs.unshift(message);

        if (_this2.logs.length > _this2.setting.maxConsoleEntries) {
          _this2.logs.pop();
        }
      });
    }
  }]);

  return ConsoleService;
}();

angular.module('roscc').service('Console', ConsoleService);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var DomainsService = function () {
  function DomainsService() {
    _classCallCheck(this, DomainsService);
  }

  _createClass(DomainsService, [{
    key: 'filterAdvanced',
    value: function filterAdvanced(entry, advanced) {
      if (advanced) {
        return true;
      }

      var entryArray = entry.split('/');
      if (!entry || _.isEmpty(entryArray)) {
        return false;
      }

      // Don't show the default nodes, params, topics and services
      return !_.contains(['rosapi', 'rosbridge_websocket', 'rosout', 'rosout_agg', 'rosversion', 'run_id', 'rosdistro', 'get_loggers', 'set_logger_level'], _.last(entryArray));
    }
  }, {
    key: 'getDomains',
    value: function getDomains(array) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length > 1) {
          result.push(nameArray[1]);
        }
      });
      return _.uniq(result).sort();
    }
  }, {
    key: 'getGlobalParameters',
    value: function getGlobalParameters(array) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length === 2) {
          entry.abbr = _.last(nameArray);
          result.push(entry);
        }
      });
      return result;
    }
  }, {
    key: 'getDataForDomain',
    value: function getDataForDomain(array, domainName) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length > 1 && nameArray[1] === domainName && (entry.fetched || !entry.active) && !_.contains(nameArray, "get_loggers") && //TODO : filter nicely <3 (maybe put the rcc filter back)
        !_.contains(nameArray, "set_logger_level")) {
          entry.abbr = nameArray.slice(2).join('/');
          result.push(entry);
        }
      });
      return result;
    }
  }]);

  return DomainsService;
}();

// Filter advanced topics, services, parameters by checking the beginning capital letter


angular.module('roscc').service('Domains', DomainsService);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var QuaternionsService = function () {
  function QuaternionsService() {
    _classCallCheck(this, QuaternionsService);
  }

  _createClass(QuaternionsService, [{
    key: 'getRoll',
    value: function getRoll(q) {
      if (!q) {
        return '';
      }
      var rad = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
      return 180 / Math.PI * rad;
    }
  }, {
    key: 'getPitch',
    value: function getPitch(q) {
      if (!q) {
        return '';
      }
      var rad = Math.asin(2 * (q.w * q.y - q.z * q.x));
      return 180 / Math.PI * rad;
    }
  }, {
    key: 'getYaw',
    value: function getYaw(q) {
      if (!q) {
        return '';
      }
      var rad = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      return 180 / Math.PI * rad;
    }
  }, {
    key: 'getInit',
    value: function getInit() {
      return { w: 1, x: 0, y: 0, z: 0 };
    }
  }]);

  return QuaternionsService;
}();

// Quaternions to Euler angles converter


angular.module('roscc').service('Quaternions', QuaternionsService);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var RosService = function () {
  function RosService($rootScope, $log, $interval, $timeout, Settings, Domains) {
    var _this = this;

    _classCallCheck(this, RosService);

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
    $interval(function () {
      _this.newRosConnection();
    }, 1000 / this.setting.refresh_rate);

    $interval(function () {
      _this.loadData();
    }, 1000 / this.setting.refresh_rate);
  }

  _createClass(RosService, [{
    key: 'newRosConnection',
    value: function newRosConnection(callback) {
      var _this2 = this;

      if (this.$rootScope.isConnected || !this.setting) {
        return;
      }

      if (this.ros) {
        this.ros.close(); // Close old connection
        this.ros = false;
        return;
      }

      this.ros = new ROSLIB.Ros({ url: 'ws://' + this.setting.address + ':' + this.setting.port });

      this.ros.on('connection', function () {
        _this2.$rootScope.isConnected = true;
        _this2.isConnected = _this2.$rootScope.isConnected;
        _this2.resetData();
        _this2.loadData();
        _this2.$log.log('Successfully connected to server !');
      });

      this.ros.on('error', function () {
        _this2.$rootScope.isConnected = false;
        _this2.isConnected = _this2.$rootScope.isConnected;
        _this2.ros = false;
        _this2.$log.log('Error trying to connect to server !');
      });

      this.ros.on('close', function () {
        _this2.$rootScope.isConnected = false;
        _this2.isConnected = _this2.$rootScope.isConnected;
        _this2.$log.log('Connection to server closed !');
      });

      if (callback) {
        this.$timeout(function () {
          callback();
        }.bind(this), 1000); // [ms]
      }
    }
  }, {
    key: 'sendOrder',
    value: function sendOrder(to, data, callback) {
      if (!this.ros) return;
      var service = new ROSLIB.Service({ ros: this.ros, name: to });
      var request = new ROSLIB.ServiceRequest(data);
      service.callService(request, callback);
    }
  }, {
    key: 'publish',
    value: function publish(to, data) {
      if (!this.ros) return;
      var topic = new ROSLIB.Topic({ ros: this.ros, name: to });
      var msg = new ROSLIB.Message(data);
      topic.publish(msg);
    }
  }, {
    key: 'listen',
    value: function listen(from, callback) {
      if (!this.ros) return;
      var topic = new ROSLIB.Topic({ ros: this.ros, name: from });
      topic.subscribe(callback);
      return topic;
    }
  }, {
    key: 'getParam',
    value: function getParam(name, callback) {
      if (!this.ros) return;
      var param = new ROSLIB.Param({ ros: this.ros, name: name });
      param.get(callback);
    }
  }, {
    key: 'setParam',
    value: function setParam(name, value) {
      if (!this.ros) return;
      var param = new ROSLIB.Param({ ros: this.ros, name: name });
      param.set(value);
    }
  }, {
    key: 'resetData',
    value: function resetData() {
      var _this3 = this;

      this.data = {
        rosout: [],
        topics: [],
        nodes: [],
        parameters: [],
        services: []
      };

      //populate expected topics/services
      angular.forEach(this.$rootScope.domains, function (d) {
        angular.forEach(d.topics, function (t) {
          var name = '/' + d.name + '/' + t;
          var newT = {
            name: name,
            abbr: t,
            active: false,
            expected: true,
            isOpen: false
          };
          _this3.data.topics.push(newT);
        });

        angular.forEach(d.services, function (s) {
          var name = '/' + d.name + '/' + s;
          var newS = {
            name: name,
            abbr: s,
            active: false,
            expected: true,
            isOpen: true
          };
          _this3.data.services.push(newS);
        });
      });
    }

    // updates structure, all data, parameters, topics, services, nodes...

  }, {
    key: 'loadData',
    value: function loadData() {
      var _this4 = this;

      this.ros.getTopics(function (topics) {
        // TODO: check if type is already returned here
        angular.forEach(topics.topics, function (name) {
          var topic = _.findWhere(_this4.data.topics, { name: name });

          if (topic) {
            //to update
            topic.active = true;
          } else {
            //to add
            topic = {
              name: name,
              active: true,
              isOpen: true
            };
            _this4.data.topics.push(topic);
          }

          if (!topic.fetched) {
            _this4.ros.getTopicType(name, function (type) {
              topic.type = type;
              topic.fetched = true;
            });
          }
        });

        for (var i = _this4.data.topics.length - 1; i >= 0; i--) {
          var t = _this4.data.topics[i];
          var found = false;
          for (var y = 0; y < topics.topics.length; y++) {
            if (topics.topics[y] == t.name) found = true;
          }

          if (!found) {
            if (!t.expected) {
              _this4.data.topics.splice(i, 1);
            } else {
              t.active = false;
            }
          }
        }
      }); // end getTopics

      this.ros.getServices(function (services) {
        angular.forEach(services, function (name) {
          var service = _.findWhere(_this4.data.services, { name: name });

          if (service) {
            //to update
            service.active = true;
          } else {
            //to add
            service = {
              name: name,
              active: true,
              isOpen: true
            };
            _this4.data.services.push(service);
          }
          if (!service.fetched) {
            _this4.ros.getServiceType(name, function (type) {
              service.type = type;
              service.fetched = true;
            });
          }
        });

        for (var i = _this4.data.services.length - 1; i >= 0; i--) {
          //angular foreach not working for this
          var s = _this4.data.services[i];
          var found = false;
          for (var y = 0; y < services.length; y++) {
            if (services[y] == s.name) found = true;
          }

          if (!found) {
            if (!s.expected) {
              _this4.data.services.splice(i, 1);
            } else {
              s.active = false;
            }
          }
        }
      }); // end getServices

      this.ros.getParams(function (params) {
        //TODO : update like topics
        angular.forEach(params, function (name) {

          var param = _.findWhere(_this4.data.parameters, { name: name });
          if (!param) {
            param = { name: name };
            _this4.data.parameters.push(param);
          }

          if (!param.fetched) {
            var rosparam = new ROSLIB.Param({ ros: _this4.ros, name: name });
            rosparam.get(function (value) {
              param.value = value;
              param.fetched = true;
            });
          }
        });

        for (var i = _this4.data.parameters.length - 1; i >= 0; i--) {
          //angular foreach not working for this
          var p = _this4.data.parameters[i];

          if (!_.contains(params, p.name)) {
            _this4.data.parameters.splice(i, 1);
          }
        }
      });

      this.ros.getNodes(function (nodes) {
        //TODO : update like topics
        _this4.data.nodes = [];
        angular.forEach(nodes, function (name) {
          _this4.data.nodes.push({ name: name });
        });
      });
    }
  }, {
    key: 'getDomains',
    value: function getDomains() {
      if (!this.data) return;
      var allData = this.data.topics.concat(this.data.services, this.data.nodes, this.data.parameters);
      var domains = this.Domains.getDomains(allData);

      var expectedD = _.pluck(this.$rootScope.domains, 'name');

      //set expected domains first in the list
      return _.uniq(expectedD.concat(domains));
    }
  }, {
    key: 'getExpectedDomains',
    value: function getExpectedDomains() {
      return _.pluck(this.$rootScope.domains, 'name');
    }
  }, {
    key: 'getUnexpectedDomains',
    value: function getUnexpectedDomains() {
      return _.difference(this.getDomains(), this.getExpectedDomains());
    }
  }, {
    key: 'getTopicsForDomain',
    value: function getTopicsForDomain(domain) {
      return this.Domains.getDataForDomain(this.data.topics, domain, false);
    }
  }, {
    key: 'getServicesForDomain',
    value: function getServicesForDomain(domain) {
      return this.Domains.getDataForDomain(this.data.services, domain, false);
    }
  }, {
    key: 'getGlobalParameters',
    value: function getGlobalParameters() {
      return this.Domains.getGlobalParameters(this.data.parameters);
    }
  }]);

  return RosService;
}();

angular.module('roscc').service('Ros', RosService);
"use strict";

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

/* eslint-disable no-undef */

var HokuyoController = function HokuyoController($rootScope, $scope, Hokuyo, Settings, Ros) {
  _classCallCheck(this, HokuyoController);

  $rootScope.act_page = 'hokuyo';
  this.$scope = $scope;
  this.ros = Ros;
  this.setting = Settings.get();

  var changedTab = false;
  if (Hokuyo.displays.one != null) {
    changedTab = true;
  }

  Hokuyo.displays.one = new HokuyoDisplay("hok1", "one", true);
  Hokuyo.displays.two = new HokuyoDisplay("hok2", "two");
  Hokuyo.displays.main = new HokuyoDisplay("mainHokDisp", "main", false, Hokuyo.displays.one.dotColor, Hokuyo.displays.two.dotColor);

  if (changedTab) {
    // Restore data
    if (Hokuyo.lastData.one != null) {
      Hokuyo.displays.one.updatePolarSpots(Hokuyo.lastData.one);
    }
    if (Hokuyo.lastData.two != null) {
      Hokuyo.displays.two.updatePolarSpots(Hokuyo.lastData.two);
    }
    if (Hokuyo.lastData.main != null) {
      Hokuyo.displays.main.updateAll(Hokuyo.lastData.main.hokuyos, Hokuyo.lastData.main.robotsSpots, Hokuyo.lastData.main.cartesianSpots);
    }
  }

  this.ros.listen(this.setting.hokuyo_1, function (msg) {

    this.$scope.name = "hokuyo.polar_raw_data";
    this.$scope.from = "hokuyo";
    this.$scope.data = {
      hokuyo: "one",
      polarSpots: Hokuyo.formatPolarPoints(msg)
    };
    this.$scope.update();
  }.bind(this));

  this.ros.listen(this.setting.hokuyo_2, function (msg) {
    this.$scope.name = "hokuyo.polar_raw_data";
    this.$scope.from = "hokuyo";
    this.$scope.data = {
      hokuyo: "two",
      polarSpots: Hokuyo.formatPolarPoints(msg)
    };

    this.$scope.update();
  }.bind(this));

  //     $scope.name = "hokuyo.polar_raw_data";
  //     $scope.from = "hokuyo";
  //     $scope.data = '{\n\
  //     "hokuyo": "one",\n\
  //     "polarSpots": [\n\
  //     [ -90, 350 ],\n\
  //     [ -30, 235 ],\n\
  //     [ -35, 230 ],\n\
  //     [ -25, 230 ],\n\
  //     [ -20, 100 ],\n\
  //     [ -15, 105 ],\n\
  //     [ -5, 120 ],\n\
  //     [ 0, 100 ],\n\
  //     [ 5, 90 ],\n\
  //     [ 10, 95 ],\n\
  //     [ 15, 100 ],\n\
  //     [ 20, 100 ],\n\
  //     [ 25, 230 ],\n\
  //     [ 30, 235 ]\n\
  //   ]\n\
  // }';
  //
  //
  //     $scope.name = "lidar.all";
  //     $scope.from = "lidar";
  //     $scope.data = '{\n\
  //       "hokuyos": [\n\
  //         {\n\
  //           "name": "one",\n\
  //           "position": {\n\
  //             "x": -6.2,\n\
  //             "y": -6.2,\n\
  //             "w": 0\n\
  //           }\n\
  //         },\n\
  //         {\n\
  //           "name": "two",\n\
  //           "position": {\n\
  //             "x": 306.2,\n\
  //             "y": 100,\n\
  //             "w": 180\n\
  //           }\n\
  //         }\n\
  //       ],\n\
  //       "cartesianSpots": [\n\
  //         [ 20, 200 ],\n\
  //         [ 30, 202 ],\n\
  //         [ 40, 199 ],\n\
  //         [ 148, 104 ],\n\
  //         [ 145, 95 ],\n\
  //         [ 153, 105 ],\n\
  //         [ 155, 98 ],\n\
  //         [ 95, 135 ],\n\
  //         [ 100, 129 ],\n\
  //         [ 101, 140 ],\n\
  //         [ 105, 132 ],\n\
  //         [ 104, 137 ],\n\
  //         [ 230, 2 ],\n\
  //         [ 234, 4 ]\n\
  //       ],\n\
  //       "robotsSpots": [\n\
  //         [ 150, 100 ],\n\
  //         [ 100, 135 ]\n\
  //       ]\n\
  //     }';

  $scope.update = function () {
    Hokuyo.onOrder($scope.from, $scope.name, $scope.data);
  };

  $scope.update();
};

angular.module('roscc').component('ccHokuyo', {
  templateUrl: 'app/hokuyo/hokuyo.html',
  controller: HokuyoController
});

/* eslint-enable no-undef */
'use strict';

angular.module('roscc').service('Hokuyo', ['$rootScope', '$sce', 'Ros', function () {
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

  // format laserscan ros msg to a list of lists [theta, r], theta in degrees
  // r in cm
  this.formatPolarPoints = function (msg) {
    var amin = msg.angle_min;
    var ainc = msg.angle_increment;

    var acurr = amin;
    var out = [];
    var _iteratorNormalCompletion = true;
    var _didIteratorError = false;
    var _iteratorError = undefined;

    try {
      for (var _iterator = msg.ranges[Symbol.iterator](), _step; !(_iteratorNormalCompletion = (_step = _iterator.next()).done); _iteratorNormalCompletion = true) {
        var r = _step.value;

        out.push([acurr * 180 / Math.PI % 360, r * 100]);
        acurr += ainc;
      }
    } catch (err) {
      _didIteratorError = true;
      _iteratorError = err;
    } finally {
      try {
        if (!_iteratorNormalCompletion && _iterator.return) {
          _iterator.return();
        }
      } finally {
        if (_didIteratorError) {
          throw _iteratorError;
        }
      }
    }

    return out;
  };

  this.init = function () {
    //Client.order(this.onOrder); //FIXME
  };

  this.onOrder = function (from, name, data) {
    // if($rootScope.act_page == 'hokuyo') {
    if (name == 'hokuyo.polar_raw_data') {
      if (this.displays[data.hokuyo]) {
        // Save for later
        this.lastData[data.hokuyo] = data.polarSpots;

        // console.log("Received data from " + data.hokuyo);

        // Show
        if (!this.displays[data.hokuyo].isBusy) {
          this.displays[data.hokuyo].updatePolarSpots(data.polarSpots);
        }
      }
    } else if (name == 'lidar.all' && !!this.displays.main) {
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
    } else if (name == 'lidar.light' && !!this.displays.main) {
      // Save for later
      this.lastData.main = {};
      this.lastData.main.hokuyos = data.hokuyos;
      this.lastData.main.robotsSpots = data.robotsSpots;

      // console.log("Received all");

      // Show
      if (!this.displays.main.isBusy) {
        this.displays.main.updateAll(data.hokuyos, data.robotsSpots, null);
      }
    } else if (name == 'lidar.robots' && !!this.displays.main) {}
    // this.displays.main.updateRobots(data.robots);

    // }
  }.bind(this);
}]);
"use strict";

/* eslint-disable */

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var HokuyoDisplay = function () {
  function HokuyoDisplay(parentId, mode) {
    var reinitColor = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : false;
    var oneColor = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : null;
    var twoColor = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    _classCallCheck(this, HokuyoDisplay);

    this.MAIN = "main";
    this.ONE = "one";
    this.TWO = "two";

    this.dotRadius = 1; // cm

    this.parentId = parentId;
    this.mode = mode;
    this.div = $('#' + this.parentId);

    this.isBusy = false; // lock

    this.clearMainTimeout = null; // timeout in case we don't receive data for a while

    if (reinitColor) {
      Raphael.getColor.reset();
    }

    this.onResize(); // Math.max($('body').height() - $('#div_menu').outerHeight() - 2*$('#simu_before').outerHeight(), 200);

    // On window resize (doesn't seem to work)
    // window.addEventListener('resize', this.onResize());
    // this.div.resize(this.onResize());
    // setInterval(this.onResize(), 1000);

    this.initChart(oneColor, twoColor);
  }

  _createClass(HokuyoDisplay, [{
    key: "onResize",
    value: function onResize() {
      this.realW = 0;
      this.realH = 0;
      this.W = 0;
      this.H = 0;
      this.center = {};
      this.center.x = 0;
      this.center.y = 0;
      this.center.str = this.center.x + "," + this.center.y; // in the viewport frame
      this.viewportScale = 1;

      if (this.mode == this.MAIN) {
        var maxWidth = 1000; // px

        this.realW = 300;
        this.realH = 200;

        // if (this.W != this.div.width()) {
        this.W = 1000; //this.div.width();
        if (this.W > maxWidth) {
          this.W = maxWidth;
        }
        this.H = this.realH / this.realW * this.W;

        // this.center.x = 0; // see initChart()
        // this.center.y = 0; // see initChart()

        // }

        var scaleTable = 0.9;

        // Center
        this.center.x = Math.round((1 - scaleTable) * this.W / 2);
        this.center.y = Math.round((1 - scaleTable) * this.H / 2);

        // viewport scale : position (cm) * scale = position (pixel)
        // we keep space for the outer towers
        this.viewportScale = scaleTable * this.W / this.realW;
      } else {
        this.realW = 400;
        this.realH = this.realW;

        this.W = 490;
        this.H = this.W;

        this.center.x = this.W / 2;
        this.center.y = this.H / 2;

        this.viewportScale = this.W / 2 / this.realW;
      }

      this.center.str = this.center.x + "," + this.center.y;

      this.div.width(this.W);
      this.div.height(this.H);

      // console.log(this.viewportScale);
      // console.log(this.center.str);
      // console.log(this.W);
      // console.log(this.H);
    }
  }, {
    key: "initChart",
    value: function initChart() {
      var oneColor = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;
      var twoColor = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;

      this.isBusy = true;

      this.r = Raphael(this.parentId, this.div.width(), this.div.height());

      var grey = Raphael.color("#333");

      if (this.mode != this.MAIN) {

        this.dotColor = Raphael.getColor(1);

        // Outmost circle
        // Unit: cm
        this.r.circle(0, 0, this.realW).attr({ stroke: this.dotColor, fill: grey, transform: "t " + this.center.str + "s" + this.viewportScale, "fill-opacity": .1 });

        // Inner circles
        // Unit: cm
        for (var radius = 100; radius < this.realW; radius += 100) {
          this.r.circle(0, 0, radius).attr({ stroke: grey, fill: null, transform: "t " + this.center.str + "s" + this.viewportScale, "stroke-opacity": .4 });
        }

        // Arraw
        // Unit: pixel
        this.r.path("M-7 10L0 -10L7 10L-7 10").attr({ stroke: this.dotColor, fill: this.dotColor, transform: "t " + this.center.str, "fill-opacity": .4 });

        this.dots = new Map();
      } else {
        // Init main

        if (oneColor == null || twoColor == null) {
          console.error("Main display must have the hokuyos Raphael colors");
          this.isBusy = false;
          return;
        }

        this.oneColor = oneColor;
        this.twoColor = twoColor;

        this.dotColor = Raphael.color("#11B100");

        // Field
        // Paper.rect(x, y, width, height, [r]) in the REAL WORLD dimensions
        // Transform : move to origin and then scale around origin
        this.r.rect(0, 0, this.realW, this.realH).attr({ stroke: grey, fill: grey, transform: "t " + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0", "fill-opacity": .1, "stroke-opacity": .4 });

        // Towers
        // 8*8 sqare at the corner - 2.2cm of wood board
        var xs = [-10.2, 302.2];
        var ys = [-10.2, 96, 202.2];
        var colors = [Raphael.color("#2988D8"), Raphael.color("#FAFC08")];
        var i = 0;

        // for each column (ie left & right)
        for (var col = 0; col <= 1; col++) {
          // for each line (ie back, middle, front)
          for (var line = 0; line <= 2; line++) {
            this.r.rect(xs[col], ys[line], 8, 8).attr({ stroke: colors[i % 2], fill: colors[i % 2], transform: "t " + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0", "fill-opacity": .4, "stroke-opacity": .8 });
            i++;
          }
        }

        // Hokuyos container
        this.hokuyos = {};

        // Dots container
        this.objects = [];
      }

      this.isBusy = false;
    }
  }, {
    key: "updatePolarSpots",
    value: function updatePolarSpots(spots) {
      if (this.mode == this.MAIN) {
        console.error("Main display can't handle polar spot");
        return;
      }

      this.isBusy = true;

      // console.log(spots);
      // For each spots
      spots.forEach(function (newSpot) {
        var existingPlot = this.dots.get(newSpot[0]);

        if (!!existingPlot) {
          // Dot already exist at this angle, let's update it
          existingPlot.attr({ cy: newSpot[1] * this.viewportScale });
        } else {
          // This dot doesn't already exist, let's create it
          var dot = this.r.circle(0, newSpot[1] * this.viewportScale, this.dotRadius).attr({
            stroke: this.dotColor,
            fill: this.dotColor,
            transform: "t," + this.center.str + "r180,0,0" + "r" + -newSpot[0] + ",0,0" });
          //  + " 0 0" +   +
          this.dots.set(newSpot[0], dot);
        }
      }.bind(this));

      this.isBusy = false;
    }

    // updateRobots(robots) {
    // 	if (this.mode != this.MAIN) {
    // 		console.error("Single LiDAR displays can't handle global robot spot");
    // 		return;
    // 	}
    // }

  }, {
    key: "updateAll",
    value: function updateAll(hokuyos, robots, cartesianSpots) {
      if (this.mode != this.MAIN) {
        console.error("Single LiDAR displays can't handle global robot spot");
        return;
      }

      this.isBusy = true;

      if (!!this.clearMainTimeout) {
        clearTimeout(this.clearMainTimeout);
      }

      // console.log(hokuyos);

      // For each hokuyo
      hokuyos.forEach(function (hok) {
        var existingHok = this.hokuyos[hok.name];

        if (!!existingHok) {
          existingHok.remove();
        }

        var color = hok.name == "one" ? this.oneColor : this.twoColor;
        this.hokuyos[hok.name] = this.r.path("M-5 3L5 0L-5 -3L-5 3").attr({
          stroke: color,
          fill: color,
          transform: "t " + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0" + "t " + hok.position.x + "," + hok.position.y + "r" + hok.position.w,
          "fill-opacity": .4 });
      }.bind(this));

      // For each object
      var _iteratorNormalCompletion = true;
      var _didIteratorError = false;
      var _iteratorError = undefined;

      try {
        for (var _iterator = this.objects[Symbol.iterator](), _step; !(_iteratorNormalCompletion = (_step = _iterator.next()).done); _iteratorNormalCompletion = true) {
          var obj = _step.value;

          obj.remove();
        }
      } catch (err) {
        _didIteratorError = true;
        _iteratorError = err;
      } finally {
        try {
          if (!_iteratorNormalCompletion && _iterator.return) {
            _iterator.return();
          }
        } finally {
          if (_didIteratorError) {
            throw _iteratorError;
          }
        }
      }

      this.objects = [];

      var _iteratorNormalCompletion2 = true;
      var _didIteratorError2 = false;
      var _iteratorError2 = undefined;

      try {
        for (var _iterator2 = robots[Symbol.iterator](), _step2; !(_iteratorNormalCompletion2 = (_step2 = _iterator2.next()).done); _iteratorNormalCompletion2 = true) {
          var robot = _step2.value;

          this.objects.push(this.r.circle(robot[0], robot[1], this.dotRadius * 5).attr({
            stroke: this.dotColor,
            fill: this.dotColor,
            transform: "t," + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0",
            "fill-opacity": .4 }));
          this.objects.push(this.r.text(robot[0], robot[1], "[" + parseInt(robot[0]) + "; " + parseInt(robot[1]) + "]").attr({
            fill: this.dotColor,
            "font-size": "6px",
            transform: "t,50,25" + "t," + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0" }));
        }

        // console.log(cartesianSpots);
      } catch (err) {
        _didIteratorError2 = true;
        _iteratorError2 = err;
      } finally {
        try {
          if (!_iteratorNormalCompletion2 && _iterator2.return) {
            _iterator2.return();
          }
        } finally {
          if (_didIteratorError2) {
            throw _iteratorError2;
          }
        }
      }

      if (!!cartesianSpots) {
        var _iteratorNormalCompletion3 = true;
        var _didIteratorError3 = false;
        var _iteratorError3 = undefined;

        try {
          for (var _iterator3 = cartesianSpots[Symbol.iterator](), _step3; !(_iteratorNormalCompletion3 = (_step3 = _iterator3.next()).done); _iteratorNormalCompletion3 = true) {
            var spot = _step3.value;

            this.objects.push(this.r.circle(spot[0], spot[1], this.dotRadius).attr({
              stroke: this.dotColor,
              fill: this.dotColor,
              transform: "t," + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0",
              "fill-opacity": .4 }));
          }
        } catch (err) {
          _didIteratorError3 = true;
          _iteratorError3 = err;
        } finally {
          try {
            if (!_iteratorNormalCompletion3 && _iterator3.return) {
              _iterator3.return();
            }
          } finally {
            if (_didIteratorError3) {
              throw _iteratorError3;
            }
          }
        }
      }

      this.isBusy = false;

      this.clearMainTimeout = setTimeout(function () {
        //this.clearMain();
      }.bind(this), 1000);
    }
  }, {
    key: "clearMain",
    value: function clearMain() {
      // For each object
      for (var hokName in this.hokuyos) {
        this.hokuyos[hokName].remove();
      }

      // For each object
      var _iteratorNormalCompletion4 = true;
      var _didIteratorError4 = false;
      var _iteratorError4 = undefined;

      try {
        for (var _iterator4 = this.objects[Symbol.iterator](), _step4; !(_iteratorNormalCompletion4 = (_step4 = _iterator4.next()).done); _iteratorNormalCompletion4 = true) {
          var obj = _step4.value;

          obj.remove();
        }
      } catch (err) {
        _didIteratorError4 = true;
        _iteratorError4 = err;
      } finally {
        try {
          if (!_iteratorNormalCompletion4 && _iterator4.return) {
            _iterator4.return();
          }
        } finally {
          if (_didIteratorError4) {
            throw _iteratorError4;
          }
        }
      }
    }
  }]);

  return HokuyoDisplay;
}();

/* eslint-enable */
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var NavbarController = function () {
  function NavbarController($location, Ros) {
    _classCallCheck(this, NavbarController);

    this.$location = $location;
    this.ros = Ros;
  }

  _createClass(NavbarController, [{
    key: 'isPath',
    value: function isPath(path) {
      return this.$location.path() === path;
    }
  }]);

  return NavbarController;
}();

angular.module('roscc').component('ccNavbar', {
  templateUrl: 'app/navbar/navbar.html',
  controller: NavbarController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var ParameterController = function () {
  function ParameterController($timeout, Ros) {
    _classCallCheck(this, ParameterController);

    this.ros = Ros;
    this.$timeout = $timeout;
  }

  _createClass(ParameterController, [{
    key: '$onInit',
    value: function $onInit() {
      this.$timeout(function () {
        this.param = new ROSLIB.Param({ ros: this.ros.ros, name: this.parameter.name });
      }.bind(this), 500);
    }
  }, {
    key: 'setValue',
    value: function setValue(value) {
      this.param.set(value);
    }
  }]);

  return ParameterController;
}();

angular.module('roscc').component('ccParameter', {
  bindings: { parameter: '=' },
  templateUrl: 'app/parameters/parameters.html',
  controller: ParameterController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var ServiceController = function () {
  function ServiceController($scope, $http, $timeout, Ros) {
    _classCallCheck(this, ServiceController);

    this.$scope = $scope;
    this.$http = $http;
    this.ros = Ros;
    this.$timeout = $timeout;
    this.flashState = -1;
  }

  _createClass(ServiceController, [{
    key: '$onInit',
    value: function $onInit() {
      var _this = this;

      this.$scope.$watchGroup(['$ctrl.service.type', '$ctrl.service.active'], function () {
        _this.setFileName();
      }, function () {});
    }
  }, {
    key: 'setFileName',
    value: function setFileName() {
      var _this2 = this;

      var path = 'app/services/';

      this.fileName = path + 'default.html';

      if (!this.service.active) {
        this.fileName = path + 'disabled.html';
        return;
      } else if (!this.service.type) {
        this.fileName = path + 'default.html';
        return;
      }

      var fileName = '' + path + this.service.type + '.html';
      this.$http.get(fileName).then(function (result) {
        if (result.data) {
          _this2.fileName = fileName;
        }
      }, function () {});
    }
  }, {
    key: 'callService',
    value: function callService(input, isJSON) {
      var _this3 = this;

      //(!this.service.active)
      //  return;

      var data = isJSON ? angular.fromJson(input) : input;
      var ROSservice = new ROSLIB.Service({
        ros: this.ros.ros,
        name: this.service.name,
        serviceType: this.service.type
      });
      var request = new ROSLIB.ServiceRequest(data);

      this.flashState = -1;

      ROSservice.callService(request, function (result) {
        _this3.result = result;
        _this3.flashState = -1;
        // -1 : no flash
        // 0 : returned some object
        // 1 : return some object with a success = true
        // 2 : return some object with a success = false

        //assume this is the success state
        if (_.keys(result).length == 1 && typeof result[_.keys(result)[0]] === 'boolean') {
          var success = result[_.keys(result)[0]];
          if (success) _this3.flashState = 1;else _this3.flashState = 2;
        } else {
          _this3.flashState = 0;
        }
        if (_this3.flashBackPromise) _this3.$timeout.cancel(_this3.flashBackPromise);

        _this3.flashBackPromise = _this3.$timeout(function () {
          _this3.flashState = -1;
        }, 2000);
      });
    }
  }]);

  return ServiceController;
}();

angular.module('roscc').component('ccService', {
  bindings: { service: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: ServiceController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var SettingsController = function () {
  function SettingsController(localStorageService, Settings) {
    _classCallCheck(this, SettingsController);

    this.Settings = Settings;

    this.settings = Settings.getSettings() || [Settings.getDefaultSetting()];
    this.index = Settings.getIndex();

    if (!this.index || this.index > this.settings.length) {
      this.index = '0';
    }

    this.save(); // Save current setting again (if it's the first time)
  }

  _createClass(SettingsController, [{
    key: 'save',
    value: function save() {
      this.Settings.save(this.settings, this.index);
    }
  }, {
    key: 'add',
    value: function add() {
      this.settings.push(this.Settings.getDefaultSetting()); // Clone object
      this.index = String(this.settings.length - 1);
      this.save();
    }
  }, {
    key: 'remove',
    value: function remove() {
      this.settings.splice(this.index, 1);
      this.index = '0';

      if (!this.settings.length) {
        this.add();
      }
      this.save();
    }
  }]);

  return SettingsController;
}();

angular.module('roscc').component('ccSettings', {
  templateUrl: 'app/settings/settings.html',
  controller: SettingsController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var SettingsService = function () {
  function SettingsService($location, localStorageService) {
    _classCallCheck(this, SettingsService);

    this.$location = $location;
    this.localStorageService = localStorageService;
  }

  _createClass(SettingsService, [{
    key: 'load',
    value: function load() {
      this.index = this.localStorageService.get('selectedSettingIndex');
      this.settings = this.localStorageService.get('settings');
      if (this.settings && this.index) {
        this.setting = this.settings[this.index];
      }

      // If there are no saved settings, redirect to /settings for first setting input
      if (!this.setting) {
        this.$location.path('/settings').replace();
      }
    }
  }, {
    key: 'save',
    value: function save(newSettings, newIndex) {
      this.settings = newSettings;
      this.index = newIndex;
      this.localStorageService.set('selectedSettingIndex', newIndex);
      this.localStorageService.set('settings', newSettings);
    }
  }, {
    key: 'get',
    value: function get() {
      if (!this.setting) {
        this.load();
      }

      return this.setting;
    }
  }, {
    key: 'getIndex',
    value: function getIndex() {
      if (!this.setting) {
        this.load();
      }

      return this.index;
    }
  }, {
    key: 'getSettings',
    value: function getSettings() {
      if (!this.setting) {
        this.load();
      }

      return this.settings;
    }
  }, {
    key: 'getDefaultSetting',
    value: function getDefaultSetting() {
      return {
        name: 'Robot Name',
        address: window.location.hostname,
        port: 9090, // default port of rosbridge_server
        log: '/rosout',
        advanced: false,
        hokuyo_1: '/sensors/hokuyo_1_raw',
        hokuyo_2: '/sensors/hokuyo_2_raw',
        maxConsoleEntries: 1000,
        refresh_rate: 1,
        log_level: 2
      };
    }
  }]);

  return SettingsService;
}();

angular.module('roscc').service('Settings', SettingsService);
'use strict';

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

/* eslint-disable no-undef */

var SimulateurController = function SimulateurController($rootScope, $scope, Ros, Simulateur) {
	_classCallCheck(this, SimulateurController);

	$rootScope.act_page = 'simulateur';
	if (!Simulateur.controllerSimu) {
		console.warn('Création d\'un nouveau simulateur !');
		Simulateur.controllerSimu = new Controller("3dobjects.json", "app/simulateur/simulateur/");
		Simulateur.controllerSimu.createRenderer();
		Simulateur.controllerSimu.loadParameters();
	} else Simulateur.controllerSimu.updateRenderer();
	$scope.pos_pr = new Position();
	$scope.rot_pr = new Position();
	$scope.pos_gr = new Position();
	$scope.rot_gr = new Position();
	$scope.pos_epr = new Position();
	$scope.pos_egr = new Position();
	$scope.vueDeFace = function () {
		Simulateur.controllerSimu.selectView("front");
	};
	$scope.vueDeDessus = function () {
		Simulateur.controllerSimu.selectView("top");
	};
	$scope.vueDeDerriere = function () {
		Simulateur.controllerSimu.selectView("behind");
	};
	$scope.vueDeGauche = function () {
		Simulateur.controllerSimu.selectView("left");
	};
	$scope.vueDeDroite = function () {
		Simulateur.controllerSimu.selectView("right");
	};
	// $scope.iaJack = function () { Client.send("ia", "ia.jack"); }
	// $scope.iaPlacerPr = function () { Client.send("ia", "pr.placer"); }    //FIXME
	// $scope.iaCollisionPr = function () { Client.send("ia", "pr.collision"); }
	// $scope.iaStop = function () { Client.send("ia", "ia.stop"); }

	Simulateur.updateInterface = function () {
		$scope.pos_pr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.pr.color).position;
		$scope.rot_pr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.pr.color).rotation;
		$scope.pos_gr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.gr.color).position;
		$scope.rot_gr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.gr.color).rotation;
		$scope.pos_epr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.epr.color).position;
		$scope.pos_egr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.egr.color).position;
	};
};

angular.module('roscc').component('ccSimulateur', {
	templateUrl: 'app/simulateur/simulateur.html',
	controller: SimulateurController
});

/* eslint-enable no-undef */
"use strict";

/* eslint-disable */

//FIXME : à mettre dans le service
/**
 * Convertit la position indiquée par l'ia en celle du simulateur
 *
 * @param {Object} pos Position sent by the IA
 * @param {Number} pos.x
 * @param {Number} pos.y
 * @returns {{x: Number, z: Number}}
 */
function convertPosNew(pos) {
	var act_pos = {};
	act_pos.x = pos.x + 1.5;
	act_pos.z = pos.y + 1; // oui c'est logique !!!
	return act_pos;
}

/**
 * Met à jour les paramètres du PR avec les données envoyées par l'IA
 *
 * @param {Object} data_robot Data for the PR sent by the IA
 * @param {Number} data_robot.x
 * @param {Number} data_robot.y
 * @param {Number} data_robot.a
 * @param {Array<Number>} data_robot.path
 * @param {Object} Simulateur
 * @param {String} type Type de Robot
 */
function updateRobot(data_robot, simulateur, type) {
	if (data_robot.x && simulateur.controllerSimu.objects3d.has(type + "_" + data_robot.color)) {
		act_pos = convertPosNew(data_robot);
		position = new Position(act_pos.x, 0, act_pos.z);
		position.roundAll(-3);
		rotation = new Position(0, data_robot.a, 0);
		rotation.roundAll(-2);
		simulateur.controllerSimu.objects3d.get(type + "_" + data_robot.color).updateParams({
			pos: position,
			rotation: rotation
		});

		if (!!data_robot.path && data_robot.path.length > 1) {
			updatePath(data_robot.path, simulateur, type + "_" + data_robot.color);
		} else {
			clearPath(simulateur, type + "_" + data_robot.color);
		}
	} else {
		console.warn("Given robot not found or properly formed");
	}
}

/**
 * Met à jour le chemin du robot
 *
 * @param {Array<Number>} path
 * @param {Object} simulateur
 * @param {String} robot
 */
function updatePath(path, simulateur, robot) {
	var PATH_HIGHT = 0.1;
	simulateur.controllerSimu.objects3d.get(robot).showPath(path.map(function (pos) {
		act_pos = convertPosNew({ x: pos[0], y: pos[1] });
		return new Position(act_pos.x, PATH_HIGHT, act_pos.z);
	}));
}

/**
 * Supprime le chemin du robot
 *
 * @param {Array<Number>} path
 * @param {Object} simulateur
 * @param {String} robot
 */
function clearPath(simulateur, robot) {
	simulateur.controllerSimu.objects3d.get(robot).clearPath();
}

angular.module('roscc').service('Simulateur', ['$rootScope', 'Ros', function () {
	this.init = function () {
		/*Client.order(function (from, name, data) {
  	if (name == 'simulateur' && $rootScope.act_page == 'simulateur') {
  		this.robots = data.robots;
  			// Met à jour le pr (s'il existe)
  		updateRobot(data.robots.pr, this, "pr");
  		// Met à jour le gr (s'il existe)
  		updateRobot(data.robots.gr, this, "gr");
  			updateRobot(data.robots.epr, this, "pr");
  		updateRobot(data.robots.egr, this, "gr");
  		$rootScope.$apply();
  			this.updateInterface();
  	}
  }.bind(this));*/ //FIXME
	};
}]);

/* eslint-enable */
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var TopicController = function () {
  function TopicController($scope, $http, Settings, Quaternions, Ros) {
    _classCallCheck(this, TopicController);

    this.$scope = $scope;
    this.$http = $http;
    this.setting = Settings.get();
    this.Quaternions = Quaternions;
    this.ros = Ros;
    this.isSubscribing = false;
    this.toggle = true;
  }

  _createClass(TopicController, [{
    key: '$onInit',
    value: function $onInit() {
      var _this = this;

      this.roslibTopic = new ROSLIB.Topic({
        ros: this.ros.ros,
        name: this.topic.name,
        messageType: this.topic.type
      });

      var path = 'app/topics/';
      this.fileName = path + 'default.html';

      this.$scope.$watchGroup(['$ctrl.topic.type', '$ctrl.topic.active'], function () {
        if (!_this.topic.active) {
          _this.fileName = path + 'disabled.html';
          _this.isSubscribing = false;
          _this.toggle = false;
          return;
        } else if (!_this.topic.type) {
          _this.fileName = path + 'default.html';
          _this.toggleSubscription(false);
          return;
        }

        var fileName = '' + path + _this.topic.type + '.html';
        _this.$http.get(fileName).then(function (result) {
          if (result.data) {
            _this.fileName = fileName;
            _this.toggleSubscription(false);
          }
        }, function () {});
      });
    }
  }, {
    key: 'toggleSubscription',
    value: function toggleSubscription(data) {
      var _this2 = this;

      if (!this.topic.active) return;
      if (!data) {
        this.roslibTopic.subscribe(function (message) {
          _this2.message = message;
        });
      } else {
        this.roslibTopic.unsubscribe();
      }
      this.isSubscribing = !data;
    }
  }, {
    key: 'publishMessage',
    value: function publishMessage(input, isJSON) {
      var data = isJSON ? angular.fromJson(input) : input;
      var message = new ROSLIB.Message(data);
      this.roslibTopic.publish(message);
    }
  }]);

  return TopicController;
}();

angular.module('roscc').component('ccTopic', {
  bindings: { topic: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: TopicController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var TransformController = function () {
  function TransformController(Ros) {
    _classCallCheck(this, TransformController);

    this.ros = Ros;
  }

  _createClass(TransformController, [{
    key: '$onInit',
    value: function $onInit() {
      this.refresh();
    }
  }, {
    key: 'refresh',
    value: function refresh() {
      // relative to fixed
      console.log("Subscribed");
      if (!this.transform.fixed || !this.transform.frame) return;

      this.tfClient = new ROSLIB.TFClient({
        ros: this.ros.ros,
        fixedFrame: this.transform.fixed,
        angularThres: 0.001,
        transThres: 0.001
      });

      this.tfClient.subscribe(this.transform.frame, function (tf) {
        var eulAngles = new THREE.Euler();

        eulAngles.setFromQuaternion(new THREE.Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w));

        this.transform.response = tf;
        this.transform.response.rotation = eulAngles;
      }.bind(this));
    }
  }]);

  return TransformController;
}();

angular.module('roscc').component('ccTransform', {
  bindings: { parameter: '=' },
  templateUrl: 'app/transforms/transforms.html',
  controller: TransformController
});
/**
 * @file Controlleur du simulateur
 * @author Mindstan
 *
 * @requires THREE
 * @requires THREE.OrbitControls
 * @requires position
 * @requires object3d
 */

"use strict";

/**
 * Gère le simulateur
 */

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var Controller = function () {
    /**
     * Constructeur du controlleur
     *
     * Prend en paramètre le chemin d'accès à la configuration et le chemin d'accès aux ressources.
     *
     * @param {String} configPath Chemin d'accès au fichier de coniguration relatif à {@link Controller.ressourcesPath}
     * @param {String} ressourcesPath Chemin d'accès aux ressources appelées par le simulateur
     */
    function Controller(configPath, ressourcesPath) {
        _classCallCheck(this, Controller);

        /**
         * Chemin d'accès au fichier contenant la configuration du simulateur.
         * @see Controller#ressourcesPath
         * @type {String}
         */
        this.configPath = configPath;

        /**
         * Chemin d'accès aux différentes ressources chargées par le simulateur.
         * @type {String}
         */
        this.ressourcesPath = ressourcesPath;

        // A charger dynamiquement du fichier
        /**
         * Id de la balise qui contiendra le simulateur.
         * @type {String}
         * */
        this.container = document.getElementById("simulateur_container");

        /** @type {Map<String, Object3d>} */
        this.objects3d = new Map();

        /** @type {Array<external:THREE.DirectionalLight>} */
        this.directionLights = [];

        /**
         * @type {Number}
         * @const
         */
        this.RADIUS_PR = 0.1;

        /**
         * @type {Number}
         * @const
         */
        this.RADIUS_GR = 0.2;
    }

    /**
     * Charge tous les paramètres
     * @see {@link Controller#configPath}
     * @see {@link Controller#ressourcesPath}
     */


    _createClass(Controller, [{
        key: "loadParameters",
        value: function loadParameters() {
            var _this = this;

            getParsedJSONFromFile(this.ressourcesPath + this.configPath, function (objects) {
                _this.create3dObjects(objects);
            });
        }

        /**
         * Crée tous les objets 3D passés en paramètres
         *
         * @param {Array<Object>} objects liste de tous les objets à créer
         * @see {@link Controller#ressourcesPath}
         */

    }, {
        key: "create3dObjects",
        value: function create3dObjects(objects) {
            var _this2 = this;

            console.log("Creating the 3D objects...");
            for (var idObject = 0; idObject < objects.length; idObject++) {
                var name = objects[idObject].name;
                //console.log("Creating " + name);
                switch (objects[idObject].type) {
                    case "pr":
                        this.objects3d.set(name, new Robot(objects[idObject], this.ressourcesPath, function (pathLine) {
                            _this2.scene.add(pathLine);
                        }, this.RADIUS_PR, this.autotrash.bind(this)));
                        break;

                    case "gr":
                        this.objects3d.set(name, new Robot(objects[idObject], this.ressourcesPath, function (pathLine) {
                            _this2.scene.add(pathLine);
                        }, this.RADIUS_GR, this.autotrash.bind(this)));
                        break;

                    default:
                        this.objects3d.set(name, new Object3d(objects[idObject], this.ressourcesPath));
                }
                this.objects3d.get(name).loadMesh(function (scene) {
                    _this2.scene.add(scene);
                });
            }
        }

        /**
         * Crée le moteur de rendu
         */

    }, {
        key: "createRenderer",
        value: function createRenderer() {
            var _this3 = this;

            // hauteur de la zone de rendu
            var height = Math.max($('body').height() - $('#div_menu').outerHeight() - 2 * $('#simu_before').outerHeight(), 200);
            //alert($('body').height() - $('#div_menu').outerHeight() - 2*$('#simu_before').outerHeight());
            // largeur de la zone de rendu
            var width = $('#simulateur_container').width();

            /** @type {external:THREE.Scene} */
            this.scene = new THREE.Scene();
            /** @type {external:THREE.PerspectiveCamera} */
            this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 10);

            /** @type {external:THREE.WebGLRenderer} */
            this.renderer = new THREE.WebGLRenderer();
            this.renderer.setSize(width, height);
            this.renderer.setClearColor(0x272525, 0.5);
            this.container.appendChild(this.renderer.domElement);

            /** @type {external:THREE.OrbitControls} */
            this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);

            /** @type {external:THREE.AxisHelper} */
            this.axisHelper = new THREE.AxisHelper(5);
            this.scene.add(this.axisHelper);

            this.createLights();
            this.selectView("front");

            // Permet de changer la taille du canva de THREE en fonction de la taille de la fenêtre
            window.addEventListener('resize', function () {
                var HEIGHT = Math.max($('body').height() - $('#div_menu').outerHeight() - 2 * $('#simu_before').outerHeight(), 200);
                var WIDTH = $('#simulateur_container').width();
                _this3.renderer.setSize(WIDTH, HEIGHT);
                _this3.camera.aspect = WIDTH / HEIGHT;
                _this3.camera.updateProjectionMatrix();
            });

            // On lance le rendu
            this.render();
        }

        /**
         * Met à jour le rendu lorsque la page est rechargée par angular
         */

    }, {
        key: "updateRenderer",
        value: function updateRenderer() {
            this.container = document.getElementById("simulateur_container");
            this.container.appendChild(this.renderer.domElement);
        }

        /**
         * Crée et insert des lumières directionnelles dans la scène
         */

    }, {
        key: "createLights",
        value: function createLights() {
            var largeurTable = 2;
            var longueurTable = 3;
            var heightLights = 2;
            // Les lumières sont disposés haut-dessus des quattres coins du plateau, avec un offset de 1
            var posLights = [new Position(-largeurTable / 2, heightLights, -longueurTable / 2), new Position(largeurTable * 3 / 2, heightLights, -longueurTable / 2), new Position(-largeurTable / 2, heightLights, longueurTable * 3 / 2), new Position(largeurTable * 3 / 2, heightLights, longueurTable * 3 / 2)];

            posLights.forEach(function (pos) {
                var light = new THREE.DirectionalLight(0xffffff, 1);
                light.position.set(pos.x, pos.y, pos.z);
                light.intensity = 0.5;
                this.directionLights.push(light);
                this.scene.add(this.directionLights[this.directionLights.length - 1]);
            }, this);
        }

        /**
         * Crée une boucle et met à jour le rendu à 60fps
         */

    }, {
        key: "render",
        value: function render() {
            var _this4 = this;

            requestAnimationFrame(function () {
                _this4.render();
            });
            this.renderer.render(this.scene, this.camera);
        }
    }, {
        key: "test",
        value: function test() {
            var geometry = new THREE.BoxGeometry(1, 1, 1);
            var material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
            var cube = new THREE.Mesh(geometry, material);
            this.scene.add(cube);

            this.camera.position.z = 5;
        }

        /**
         * Affiche la vue désirée
         *
         * Valeurs possibles : "front", "top", "behind", "left", "right"
         *
         * @param {String} view Vue désirée
         */

    }, {
        key: "selectView",
        value: function selectView(view) {
            var X_MAX = 3;
            var Z_MAX = 2;
            console.log("Changement de vue : " + view);
            switch (view) {
                case "front":
                    this.controls.reset();
                    this.camera.position.set(1.5, 1.5, 3.5);
                    this.camera.rotation.set(-0.5, 0, 0);
                    this.controls.target.set(X_MAX / 2, 0, Z_MAX / 2);
                    break;

                case "top":
                    this.controls.reset();
                    this.camera.position.set(1.5, 3, 1);
                    this.camera.rotation.set(-1.6, 0, 0);
                    this.controls.target.set(X_MAX / 2, 0, Z_MAX / 2);
                    break;

                case "behind":
                    this.controls.reset();
                    this.camera.position.set(1.5, 1, -1.5);
                    this.camera.rotation.set(-2.5, 0, 3.0);
                    this.controls.target.set(X_MAX / 2, 0, Z_MAX / 2);
                    break;

                case "left":
                    this.controls.reset();
                    this.camera.position.set(-0.8, 1.5, 1);
                    this.camera.rotation.set(-1.6, -1, -1.6);
                    this.controls.target.set(X_MAX / 2, 0, Z_MAX / 2);
                    break;

                case "right":
                    this.controls.reset();
                    this.camera.position.set(4, 0.5, 1);
                    this.camera.rotation.set(-1.5, 1.5, 1.5);
                    this.controls.target.set(X_MAX / 2, 0, Z_MAX / 2);
                    break;

                default:
                    // Invalide
                    console.warn("Attention : \"" + view + "\" n'est pas une vue valide.");
            }
        }

        /**
         * Met à jour les données des objets dans param. Cette fonction est destinée à être appelée depuis le webclient.
         *
         * @param {Object} params
         */

    }, {
        key: "updateObjects",
        value: function updateObjects(params) {
            params.forEach(function (object3d) {
                this.objects3d.get(object3d.name).updateParams(object3d);
            }, this);
        }

        /**
         * Fonction destinée a être appelée par les robots pour enlever (mettre invisible) des éléments du simulateur
         *
         * @param {any} pos
         * @param {any} radius
         */

    }, {
        key: "autotrash",
        value: function autotrash(pos, radius) {
            /*if (!this.objects3d) // Pour que la fonction ne soit pas lancée lorsque la table n'est pas construite
                return;*/
            var blacklist = ["pr", "gr", "plateau"];
            this.objects3d.forEach(function (object3d) {
                if (blacklist.indexOf(object3d.type) == -1) {
                    if (object3d.position.get2dDistance(pos) <= radius && object3d.mesh) {
                        //console.log("Make invisible : " + object3d.name);
                        object3d.mesh.traverse(function (object) {
                            object.visible = false;
                        });
                    }
                }
            }, this);
        }
    }]);

    return Controller;
}();
"use strict";

/**
 * @file Défini une fonction qui télécharge via AJAX le fichier JSON, puis retourne son contenu converti en objet.
 * @author Mindstan
 * 
 * @requires {JQuery}
 */

/**
 * Télécharge via AJAX le fichier JSON demandé, puis appelle la fonction passée en paramètre avec son contenu converti en objet.
 * 
 * @param {String} fileName Nom du fichier distant
 * @param {function} onSuccess Fonction appelée lorsque la requête a abouti
 * @returns {Object}
 */
function getParsedJSONFromFile(fileName, onSuccess) {
    /* Ne fonctionne pas en local, retourne une réponse xml à la place de json
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            this.overrideMimeType('application/json');
            onSuccess(JSON.parse(this.responseText));
        }
    };
    xmlhttp.open("GET", fileName, true);
    xmlhttp.send();
    */

    // Plutôt ceci :
    $.ajax({
        dataType: "json",
        url: fileName,
        mimeType: "application/json",
        success: function success(result) {
            console.log("Successfuly retrieved " + fileName + " from the server.");
            onSuccess(result);
        }
    });
    // Fonction de JQUERY, ne fonctionne pas en local
    //$.getJSON(fileName, onSuccess);
}
/**
 * @file Gestion des objects 3d dans le simulateur
 * @author Mindstan
 * 
 * @requires THREE
 * @requires THREE.ColladaLoader
 * @requires Position
 */

"use strict";

/**
 * Gère les meshes
 * 
 */

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var Object3d = function () {
    /**
     * Constructeur de Object3d
     * 
     * @param {Object} params Paramètres divers de l'objet 3d
     * @param {String} ressourcesPath Répertoire de travail
     */
    function Object3d(params, ressourcesPath) {
        _classCallCheck(this, Object3d);

        /** @type {String} */
        this.ressourcesPath = ressourcesPath;

        /** @type {external:THREE.ColladaLoader} */
        this.loader = new THREE.ColladaLoader();
        this.loader.options.convertUpAxis = true;

        /** @type {Object} */
        this.mesh = null;

        this.setParams(params);
    }

    /**
     * Répartit les valeurs contenues dans params dans les bonnes variables membres
     * 
     * /!\ Ecrase les données existantes si elles existent ! /!\
     * 
     * @param {Object} params Paramètres divers de l'objet 3D
     */


    _createClass(Object3d, [{
        key: "setParams",
        value: function setParams(params) {
            //console.log("Loading params for " + params.name);
            /** @type {Object} */
            this.params = params;

            /** @type {String} */
            this.name = params.name;

            /** @type {Position} */
            this.position = new Position(params.pos.x, params.pos.y, params.pos.z);

            /** @type {Position} */
            this.rotation = new Position(params.rotation.x, params.rotation.y, params.rotation.z);
            this.rotation.makeRotationFromDegrees();

            /** @type {String} */
            this.source = params.source;

            /** @type {String} */
            this.color = params.color;

            /** @type {String} */
            this.type = params.type;
        }

        /**
         * Met à jour la position et la rotation de l'objet 3D.
         * Si certains paramètres ne sont pas définis, ils ne seront pas modifiés.
         * 
         * @param {Object} params Paramètres divers de l'objet 3D
         */

    }, {
        key: "updateParams",
        value: function updateParams(params) {
            if (params.pos) this.setPosition(params.pos);
            if (params.rotation) this.setRotation(params.rotation);
        }

        /**
         * Charge le mesh contenu dans le fichier collada spécifié par {@link Object3d#source}
         * 
         * Appelle la fonction onSucess avec en paramètre la scène (c'est-à-dire l'objet) losque le chargement est terminé.
         * 
         * @param {function} onSuccess Callback appelé lorsque le chargement du collada est terminé
         */

    }, {
        key: "loadMesh",
        value: function loadMesh(onSuccess) {
            var _this = this;

            this.loader.load(this.ressourcesPath + this.source, function (collada) {
                _this.mesh = collada.scene;
                _this.mesh.position.set(_this.position.x, _this.position.y, _this.position.z);
                _this.mesh.rotation.set(_this.rotation.x, _this.rotation.y, _this.rotation.z);
                _this.mesh.scale.set(1, 1, 1);
                //this.debug_scene();
                onSuccess(_this.mesh);
            });
        }

        /**
         * Change la position de l'objet
         * 
         * @param {Position} pos Nouvelle position
         */

    }, {
        key: "setPosition",
        value: function setPosition(pos) {
            this.position = pos;
            if (this.mesh) {
                this.mesh.position.set(pos.x, pos.y, pos.z);
            }
        }

        /**
         * Change la rotation de l'objet
         * 
         * @param {Position} rotation Nouvelle rotation
         */

    }, {
        key: "setRotation",
        value: function setRotation(rotation) {
            this.rotation = rotation;
            if (this.mesh) {
                this.mesh.rotation.set(rotation.x, -rotation.y, rotation.z);
            }
        }

        /**
         * Affiche l'objet en mode fils de fer
         * (utile lorsque les textures ne chargent pas ou quand les faces sont transparentes)
         */

    }, {
        key: "debug_scene",
        value: function debug_scene() {
            this.mesh.traverse(function (object) {
                if (object.material) {
                    object.material = new THREE.MeshBasicMaterial({ wireframe: true });
                }
            });
        }
    }]);

    return Object3d;
}();
/**
 * @file Classe permettant de travailler avec des coordonnées cartésiennes et angulaires dans l'espace
 * @author Mindstan
 * 
 */

"use strict";

/**
 * Permet de travailler avec des coordonnées sur 3 dimensions
 */

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var Position = function () {

  /**
   * Constructeur de Position
   *
   * @param {?Number} x
   * @param {?Number} y
   * @param {?Number} z
   */
  function Position() {
    var x = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : 0;
    var y = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : 0;
    var z = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : 0;

    _classCallCheck(this, Position);

    /** @type {Number} */
    this.x = x;
    /** @type {Number} */
    this.y = y;
    /** @type {Number} */
    this.z = z;
  }

  /**
   * Calcule la distance sur 2 dimentions par rapport à la position passée en paramètre (x et z)
   * 
   * @param {Position} pos
   * @returns {Number}
   */


  _createClass(Position, [{
    key: "get2dDistance",
    value: function get2dDistance(pos) {
      var deltaX = this.x - pos.x;
      var deltaZ = this.z - pos.z;
      return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaZ, 2));
    }

    /**
     * Calcule la distance sur 3 dimentions par rapport à la position passée en paramètre
     * 
     * @param {Position} pos
     * @returns {Number}
     */

  }, {
    key: "get3dDistance",
    value: function get3dDistance(pos) {
      var deltaX = this.x - pos.x;
      var deltaY = this.y - pos.y;
      var deltaZ = this.z - pos.z;
      return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaZ, 2));
    }

    /**
     * Aditionne la position passée en paramètre avec celle actuelle
     * 
     * @param {Position} pos
     */

  }, {
    key: "add",
    value: function add(pos) {
      this.x += pos.x;
      this.y += pos.y;
      this.z += pos.z;
    }

    /**
     * Convertit tous les angles portés par les axes en radians
     */

  }, {
    key: "makeRotationFromDegrees",
    value: function makeRotationFromDegrees() {
      this.x = this.convertDegreesToRadians(this.x);
      this.y = this.convertDegreesToRadians(this.y);
      this.z = this.convertDegreesToRadians(this.z);
    }

    /**
     * Convertit des degrés en radians
     */

  }, {
    key: "convertDegreesToRadians",
    value: function convertDegreesToRadians(deg) {
      return deg * Math.PI / 180;
    }

    /**
     * Met à jour la position sur tous les axes
     * 
     * @param {Number} x
     * @param {Number} y
     * @param {Number} z
     */

  }, {
    key: "set",
    value: function set(x, y, z) {
      this.x = x;
      this.y = y;
      this.z = z;
    }
    /**
     * Arrondit les variable à la puissance de 10 demandée.
     * 
     * @param {Number} power10 Puissance de 10 à arrondire 
     * @memberOf Position
     */

  }, {
    key: "roundAll",
    value: function roundAll(power10) {
      this.roundVar(power10, "x");
      this.roundVar(power10, "y");
      this.roundVar(power10, "z");
    }

    /**
     * Arrondit à la puissance de 10 la variable {x, y, z}
     * 
     * @param {Number} power10 Puissance de 10
     * @param {String} var_name Nom de la variable
     * 
     * @memberOf Position
     */

  }, {
    key: "roundVar",
    value: function roundVar(power10, var_name) {
      var oldVar = this[var_name];
      var newVar = Math.round(oldVar * Math.pow(10, -power10)) / Math.pow(10, -power10);
      this[var_name] = newVar;
    }
  }]);

  return Position;
}();
/**
 * Gestion des robots
 * 
 * @author Mindstan
 */

"use strict";
/**
 * Permet une gestion plus précise des robots dans le simulateur
 * 
 * @class Robot
 * @extends {Object3d}
 */

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

var _get = function get(object, property, receiver) { if (object === null) object = Function.prototype; var desc = Object.getOwnPropertyDescriptor(object, property); if (desc === undefined) { var parent = Object.getPrototypeOf(object); if (parent === null) { return undefined; } else { return get(parent, property, receiver); } } else if ("value" in desc) { return desc.value; } else { var getter = desc.get; if (getter === undefined) { return undefined; } return getter.call(receiver); } };

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

var Robot = function (_Object3d) {
    _inherits(Robot, _Object3d);

    /**
     * Crée un nouveau robot
     * 
     * @param {Object} params 
     * @param {String} ressourcesPath
     * @param {Function} onCreateFinished Fonction appelée losrque le robot a fini d'être initialisé
     * @param {Number} radius Rayon du robot (utilisé pour enlever les objets)
     * @param {Function} autotrash Fonction appelée pour supprimer des objets proches
     */
    function Robot(params, ressourcesPath, onCreateFinished, radius, autotrash) {
        _classCallCheck(this, Robot);

        /**
         * @type {Number}
         * @const
         */
        var _this = _possibleConstructorReturn(this, (Robot.__proto__ || Object.getPrototypeOf(Robot)).call(this, params, ressourcesPath));
        // On appelle le constructeur parent


        _this.PATH_MAX_POINTS = 10;

        /**
         * Rayons représentant le robot sur le plan (xOz)
         * @type {Number}
         */
        _this.radius = radius;

        /**
         * Fonction appelée sur le controlleur lorsque le robot bouge,
         * avec en paramètre la position et le rayon du robot
         * @type {Function}
         */
        _this.autoTrash = autotrash;

        _this.initPath(onCreateFinished);
        return _this;
    }

    /**
     * Initialise l'afichage du chemin que suivra le robot
     * 
     * @param {Function} onCreateFinished
     */


    _createClass(Robot, [{
        key: "initPath",
        value: function initPath(onCreateFinished) {
            var pathGeometry = new THREE.Geometry();
            var pathMaterial = new THREE.LineBasicMaterial({
                color: 0x000000,
                linewidth: 3
            });

            for (var idVertice = 0; idVertice < this.PATH_MAX_POINTS; idVertice++) {
                pathGeometry.vertices.push(new THREE.Vector3(0, 0, 0));
            } /** @type {external:THREE.Line} */
            this.pathLine = new THREE.Line(pathGeometry, pathMaterial);
            onCreateFinished(this.pathLine); // Callback
        }

        /**
         * Met à jour et affiche le chemin passé en paramètre
         * 
         * @param {Array<Position>} newPath
         */

    }, {
        key: "showPath",
        value: function showPath(newPath) {
            //console.log(newPath);
            var idVertice = 0;
            while (idVertice < newPath.length && idVertice < this.PATH_MAX_POINTS) {
                var pos = newPath[idVertice];
                var v = new THREE.Vector3(pos.x, pos.y, pos.z);
                this.pathLine.geometry.vertices[idVertice] = v;
                idVertice++;
            }
            var lastVertice = newPath[newPath.length - 1];
            while (idVertice < this.PATH_MAX_POINTS) {
                this.pathLine.geometry.vertices[idVertice] = lastVertice;
                idVertice++;
            }
            this.pathLine.geometry.verticesNeedUpdate = true;
        }

        /**
         * Supprime le chemin de ce robot
         */

    }, {
        key: "clearPath",
        value: function clearPath() {
            for (var idVertice = 0; idVertice < this.PATH_MAX_POINTS; idVertice++) {
                this.pathLine.geometry.vertices[idVertice] = new THREE.Vector3(0, 0, 0);
            }

            this.pathLine.geometry.verticesNeedUpdate = true;
        }

        /**
         * Met à jour la position et la rotation de l'objet 3D.
         * Si certains paramètres ne sont pas définis, ils ne seront pas modifiés.
         * Supprime automatiquement les objets proches.
         * 
         * @param {Object} params Paramètres divers de l'objet 3D
         */

    }, {
        key: "updateParams",
        value: function updateParams(params) {
            _get(Robot.prototype.__proto__ || Object.getPrototypeOf(Robot.prototype), "updateParams", this).call(this, params);
            this.autoTrash(this.position, this.radius);
        }
    }]);

    return Robot;
}(Object3d);
/**
 * La bibliothèque THREE
 * @external THREE
 * @see {@link https://threejs.org/docs/index.html}
 */

/**
 * Affiche des axes dans le rendu 3D
 * @class AxisHelper
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Reference/Helpers/AxisHelper}
 */

/**
 * Charge des collada
 * 
 * /!\Utilisation de la v1 du fichier, v2 en cours de développement en 2017 /!\
 * @class ColladaLoader
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Examples/Loaders/ColladaLoader}
 */

/**
 * Crée une lumière directionnelle
 * @class DirectionalLight
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Reference/Lights/DirectionalLight}
 */

/**
 * Crée une forme géométrique
 * @class Geometry
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Reference/Core/Geometry}
 */

/**
 * Crée une figure composée d'une suite de points reliés entre eux (sauf le premier et le dernier)
 * @class Line
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Reference/Objects/Line}
 */

/**
 * Crée le materiaux qui sera appliqué à une ligne
 * @class LineBasicMaterial
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Reference/Objects/Line}
 */

/**
 * Crée un controlleur pour bouger la caméra
 * @class OrbitControls
 * @memberof external:THREE
 * @see {@link https://github.com/mattdesl/three-orbit-controls}
 */

/**
 * Crée une vue perspective
 * @class PerspectiveCamera
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Reference/Cameras/PerspectiveCamera}
 */

/**
 * Crée une nouvelle scène
 * @class Scene
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Reference/Scenes/Scene}
 */

/**
 * Crée une intance de WebGL. Attention à la compatibilité des navigateurs !
 * @class WebGLRenderer
 * @memberof external:THREE
 * @see {@link https://threejs.org/docs/index.html#Reference/Renderers/WebGLRenderer}
 */
"use strict";