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


angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule', 'chart.js']).config(ROSCCConfig).run(run);
