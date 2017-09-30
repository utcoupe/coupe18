(function () {
	"use strict";
	var log4js = require('log4js');
	var logger = log4js.getLogger('ia.utils.getStartpoints');
	
	/*
		Returns 8 entry positions around the point (x,y) at the distance of d mm

		Be carefull : position is given from the top left corner, angle counterclockwise
			(0° is from left to right)


	*/

	var x = process.argv[2];
	var y = process.argv[3];

	if (!x || !y){
		logger.error("use : node getStartpoints.js x y");
		return;
	}

	var d = 150; // mm
	var d2 = Math.round(Math.sqrt(2)/2*d);

	console.log("{\n\t\"x\": "+x+",\n\t\"y\": "+eval(y+"-"+d)+",\n\t\"a\": -90\n},"); // haut
	console.log("{\n\t\"x\": "+eval(x+"+"+d2)+",\n\t\"y\": "+eval(y+"-"+d2)+",\n\t\"a\": -135\n},");
	console.log("{\n\t\"x\": "+eval(x+"+"+d)+",\n\t\"y\": "+y+",\n\t\"a\": 180\n},");
	console.log("{\n\t\"x\": "+eval(x+"+"+d2)+",\n\t\"y\": "+eval(y+"+"+d2)+",\n\t\"a\": 135\n},");
	console.log("{\n\t\"x\": "+x+",\n\t\"y\": "+eval(y+"+"+d)+",\n\t\"a\": 90\n},"); // bas
	console.log("{\n\t\"x\": "+eval(x+"-"+d2)+",\n\t\"y\": "+eval(y+"+"+d2)+",\n\t\"a\": 45\n},");
	console.log("{\n\t\"x\": "+eval(x+"-"+d)+",\n\t\"y\": "+y+",\n\t\"a\": 0\n},");
	console.log("{\n\t\"x\": "+eval(x+"-"+d2)+",\n\t\"y\": "+eval(y+"-"+d2)+",\n\t\"a\": -45\n}");

})();
