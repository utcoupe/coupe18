function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider
    .when('/reseau', { template: '<cc-reseau></cc-reseau>' })
    .when('/diagnostic', { template: '<cc-diagnostic></cc-diagnostic>' })
    .when('/asserv', { template: '<cc-asserv></cc-asserv>' })
    .when('/simulateur', { template: '<cc-simulateur></cc-simulateur>' })
    .when('/hokuyo', { template: '<cc-hokuyo></cc-hokuyo>' })
    .when('/telecommande', { template: '<cc-control></cc-control>' })
    .when('/settings', { template: '<cc-settings></cc-settings>' })
    .otherwise({ redirectTo: '/telecommande' });

  localStorageServiceProvider
    .setPrefix('roscc');
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule', 'chart.js']).config(ROSCCConfig);
