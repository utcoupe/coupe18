/**
 * Main module
 *
 * @module clients/pr/main
 * @requires server/socket_client
 * @requires config
 * @requires clients/pr/actuators
 * @requires clients/pr/detect
 * @requires grobot.class.js
 */
(function () {

	"use strict";

	var Grobot = (require('../robot/gr'));
    var grobot = new Grobot('gr');
  //grobot.sendChildren(tibot.grobot);

	grobot.start();
})();
