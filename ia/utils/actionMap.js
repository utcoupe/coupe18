(function () {
	"use strict";
	/*
		Shows a map of the differents Startpoints

	*/

	var canvas = $('#map');
	var ctx = canvas[0].getContext("2d");

	function getColor(){
		var randR = Math.round(Math.random()*256).toString(16);
		randR = (randR.length == 1)?("a"+ randR):randR;
		var randG = Math.round(Math.random()*256).toString(16);
		randG = (randG.length == 1)?("a"+ randG):randG;
		var randB = Math.round(Math.random()*256).toString(16);
		randB = (randB.length == 1)?("a"+ randB):randB;
		return "#" + randR + randG + randB;
	}

	function drawArrow(x, y, angle, color){
		/*

		--------------> 2
		|
		|
		|
		\/ 1

		*/
		// var lside = [y-10, x-5];
		// var rside = [y-10, x+5];
		// var center = [y-4, x];
		x = 3*x;
		y = 3*y;
		var lside = [y - 12*Math.cos((angle + 30)*Math.PI/180), x + 12*Math.sin((angle + 30)*Math.PI/180)];
		var rside = [y - 12*Math.cos((angle - 30)*Math.PI/180), x + 12*Math.sin((angle - 30)*Math.PI/180)];
		var center = [y - 5*Math.cos(angle*Math.PI/180), x + 5*Math.sin(angle*Math.PI/180)];

		ctx.beginPath();
		ctx.moveTo(y,x);
		ctx.lineTo(lside[0], lside[1]);
		ctx.lineTo(center[0], center[1]);
		ctx.lineTo(rside[0], rside[1]);
		ctx.lineTo(y,x);
		ctx.fillStyle = color;
		ctx.fill();
		ctx.closePath();
	}

	function drawMap(actions) {
		// draw(50,50,"red");
		// drawArrow(120, 340, 0, getColor());
		var pt, color;

		Object.keys(actions).forEach(function(action_name) {
			color = getColor();
			// console.log("Printing "+actions[action_name].startpoints.length+" sp");
			for(var i=0; i<actions[action_name].startpoints.length; i++){
				pt = actions[action_name].startpoints[i];
				// console.log(pt.x/10, pt.y/10, pt.a);
				drawArrow(pt.y/10, pt.x/10, pt.a, color);
			}
		});
	}

	function importActions(callback){
		var oReq = new XMLHttpRequest();

		oReq.onload = function(){
			var req = JSON.parse(this.response);
			callback(req.actions);
		};
		oReq.open("get", '../actions.json', true);
		oReq.send();
	}

	importActions(drawMap);
})();
