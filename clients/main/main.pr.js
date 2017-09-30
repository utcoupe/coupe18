/**
 * Main module
 *
 * @module clients/tibot/main_tibot
 * @requires clients/Robots/tibot
 */

 //In previous version, this "main" was encapsulated by a function (function () {})();

(function () {
	"use strict";

	const Tibot = require('../robot/pr');

	var tibot = new Tibot('pr');
	//tibot.sendChildren(tibot.lastStatus);
	tibot.start();
})();
