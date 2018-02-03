function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider
    .when('/reseau', { template: '<cc-reseau></cc-reseau>' })
    .when('/diagnostic', { template: '<cc-diagnostic></cc-diagnostic>' })
    .when('/asserv', { template: '<cc-asserv></cc-asserv>' })
    .when('/simulateur', { template: '<cc-simulateur></cc-simulateur>' })
    .when('/hokuyo', { template: '<cc-hokuyo></cc-hokuyo>' })
    .when('/telecommande', { template: '<cc-control></cc-control>' })
    .when('/settings', { template: '<cc-settings></cc-settings>' })
    .otherwise({ redirectTo: '/diagnostic' });

  localStorageServiceProvider
    .setPrefix('roscc');
}

function run($rootScope) {

  $rootScope.domains = [
    {
      name: 'drivers',
      topics: ['ard_asserv/pose2d',
               'ard_asserv/speed'],
      services: ['ard_asserv/emergency_stop',
                 'ard_asserv/goto',
                 'ard_asserv/set_pos',
                 'ard_asserv/speed',
                 'ard_asserv/pwm',
                 'ard_asserv/management',
                 'ard_asserv/parameters']
    },
    {
      name: 'ai',
      topics: ['oui1', 'non1'],
      services: ['s1', 's2']
    },
    {
      name: "recognition",
      topics: ['oui1', 'non1'],
      services: ['gzfd']
    },
    {
      name: "processing",
      topics: ['oui1', 'non1'],
      services: ['etr']
    },
    {
      name: "sensors",
      topics: ['oui1', 'non1'],
      services: ['trezz']
    },
    {
      name: "movement",
      topics: ['oui1', 'non1'],
      services: []
    },
    {
      name: "memory",
      topics: ['oui1', 'non1'],
      services: []
    },
    {
      name: "feedback",
      topics: ['oui1', 'non1'],
      services: []
    },
    {
      name: "navigation",
      topics: ['oui1', 'non1'],
      services: []
    }
  ];
}


angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule', 'chart.js', 'ngAnimate']).config(ROSCCConfig).run(run);
