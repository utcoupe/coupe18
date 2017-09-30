function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider
    .when('/reseau', { template: '<cc-reseau></cc-reseau>' })
    .when('/simulateur', { template: '<cc-simulateur></cc-simulateur>' })
    .when('/hokuyo', { template: '<cc-hokuyo></cc-hokuyo>' })
    .when('/diagnostic', { template: '<cc-diagnostic></cc-diagnostic>' })
    .when('/telecommande', { template: '<cc-control></cc-control>' })
    .when('/settings', { template: '<cc-settings></cc-settings>' })
    .otherwise({ redirectTo: '/diagnostic' });

  localStorageServiceProvider
    .setPrefix('roscc');
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule']).config(ROSCCConfig);
