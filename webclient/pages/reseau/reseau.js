/*

Je suis désolé pour ce code très très moche... J'étais jeune et insouciant O:)
Mewen - 03/2017

*/


angular.module('app').controller('ReseauCtrl', ['$rootScope', '$scope', 'Reseau',
	function($rootScope, $scope, Reseau) {
	$rootScope.act_page = 'reseau';
	Reseau.updateLayout(Reseau.network);
}]);

angular.module('app').service('Reseau', ['$rootScope', 'Client', function($rootScope, Client) {
	this.network = {};
	var links = [];

	/* ------------- Page reactions ------------- */

	$(document).on("click", ".hokuyo", function(e) {
		switch(e.target.innerHTML){
			// case "Start":
			// 	Client.send("hokuyo", "start", {
			// 		"color": $("#rc_hok_color").val()});
			// 	break;
			// case "Stop":
			// 	Client.send("hokuyo", "stop", {});
			// 	break;
			case "Shutdown":
			Client.send("hokuyo", "shutdown", {});
				break;
			default:
				console.warn("Unknown click content");
		}
	});

	$(document).on("click", ".pr", function(e) {
		if(e.target.innerHTML == "Start"){
			Client.send("pr", "start", {});
		} else {
			Client.send("pr", "stop", {});
		}
	});

	$(document).on("click", ".gr", function(e) {
		if(e.target.innerHTML == "Start"){
			Client.send("gr", "start", {});
		} else {
			Client.send("gr", "stop", {});
		}
	});

	// $(document).on("click", ".lidar", function(e) {
	// 	if(e.target.innerHTML == "yellow"){
	// 		Client.send("lidar", "start", {
	// 				"color": "yellow"
	// 			});
	// 	} else if(e.target.innerHTML == "blue"){
	// 		Client.send("lidar", "start", {
	// 				"color": "blue"
	// 			});
	// 	} else {
	// 		Client.send("lidar", "stop", {});
	// 	}
	// });


	/* --------- Prints ------------- */
		function addDiv (parentId, currentId, type, color, name, ip) {
		    // possible colors : error, green, yellow, waiting, ok, starting, everythingIsAwesome or normal (just "" )

		    var newDiv = document.createElement('div');

		    var more = "";
		    if (parentId == "webclients") 
		        more = " webclient";
		    else if (parentId == "clients") 
		        more = " client";
		    else if ((parentId == "brain") && (type != "server"))
		        more = " brain";
		    more += " " + color;

		    newDiv.id    = currentId;
		    newDiv.title = "ID de l'appareil : " + currentId + "\nStatus : "+color;
		    newDiv.setAttribute('class', "thing device" + type + more);

		    newDiv.innerHTML = "<h3>" + name + "</h3>";
		    switch(color){
		    	case "waiting":
		    		newDiv.innerHTML += "<span>En attente d'un 'start'...</span><br/>";
		    		break;
		    	case "ok":
		    		newDiv.innerHTML += "<span>Minimum vital en route</span><br/>";
		    		break;
		    	case "starting":
		    		newDiv.innerHTML += "<span>En cours de démarrage</span><br/>";
		    		break;
		    	case "everythingIsAwesome":
		    		newDiv.innerHTML += "<span>Tout va bien :)</span><br/>";
		    		break;
		    	case "error":
		    		newDiv.innerHTML += "<span>Élements vitaux manquants</span><br/>";
		    		break;
		    	default:
		    		newDiv.innerHTML += "<br/>";
		    }

		    if(parentId == "clients" || parentId == "children"){
		    	devClass = name.toLowerCase();
		    	
		    	// if(color == "waiting"){
		    	// 	// Hok params
		    	// 	if(devClass == "lidar"){
		    	// 		// newDiv.innerHTML += "<select id='rc_hok_color'> <option value='green' selected>vert</option> <option value='yellow'>jaune</option> </select>";
			    //     	newDiv.innerHTML += "Start : <button type='button' class='btn "+devClass+" yellow'>yellow</button>";
			    //     	newDiv.innerHTML += "<button type='button' class='btn "+devClass+" blue'>blue</button>";
		    	// 	} else {
			    //     	newDiv.innerHTML += "<button type='button' class='btn "+devClass+"'>Start</button>";
		    	// 	}


		    	// 	
		    	// 	else 
		    	// 		newDiv.innerHTML += "<br>";
			    // } else{
			    //     newDiv.innerHTML += "<button type='button' class='btn "+devClass+"'>Stop</button><br>";
		    	// }
		    	if(devClass == "hokuyo"){
    				newDiv.innerHTML += "<button type='button' class='btn hokuyo'>Shutdown</button><br>";
		    	}
		    }
		    
		    if(ip)
		        newDiv.innerHTML += "<span class='ip'>"+ip+"</span>";
		    // console.log(newDiv);
		    document.getElementById(parentId).appendChild(newDiv);
		}

		function printNotConnected () {
		    clearArrows();
		    var devices = document.querySelectorAll(".rects");

		    for(var i=0; i < devices.length; i++) {
		        var current = devices[i];
		        current.innerHTML = "";
		    }

		    addDiv("webclients", "owner", "laptop", "", "Toi", "127.0.0.1");
		    addDiv("brain", "server", "server", "error", "Serveur", "?");
		}



	/* --------- Layout ------------- */
		this.updateLayout = function (status) {
		    clearColumns();
		    links = [];

		    // Update content size
		    $("#page").height(0.95*($( window ).height() - $("#page").offset().top));
		    
		    // console.log(status);
		    if (!!status && !!status.server){
		        // Adds divs

		        var client, i;

		        // if (Object.keys(status.simulator).length == 1) {
		        //     addDiv("brain", "simu", "simu", "", "Simulateur", "");
		        // }

				// Webclients
					for(i in status.webclient) {
					    client = status.webclient[i];
					    addDiv("webclients", i, client.type, "", client.name, client.ip);
					}

		        addDiv("brain", "server", "server", "", "Serveur", status.server.ip);

		        // IA(s)
					for(i in status.ia) {
					    client = status.ia[i];
					    addDiv("brain", i, "ia", (client.color || ""), "IA", "");

					}

		        // Clients and their children
					var randId;
					let lidarKeys = Object.keys(status.lidar);
					if (lidarKeys.length > 1) {
						console.log("There's more than one Lidar object, we onely print the first as it is supposed to be a singleton");
					}
					if (lidarKeys.length > 0) {
						client = status.lidar[lidarKeys[0]];
					    addDiv("clients", lidarKeys[0], "lidar", client.status, "Lidar", client.ip);
					}
					
					for(i in status.hokuyo) {
					    client = status.hokuyo[i];
					    // addDiv("clients", i, "hok", client.status, "Hokuyo", client.ip);
					    addDiv("children", i, "hok", client.status, "Hokuyo", client.ip);

					    if (!!lidarKeys[0]) {
				            links.push({
				            	start: lidarKeys[0],
				            	end: i
				            });
					    }

					    // if(!!client.children)
					    //     for(var j=0; j<client.children.length; j++) {
					    //     	randId = Math.round(Math.random()*1000);
					    //         addDiv("children", (client.children[j].replace(/\s+/g,''))+randId, "hok", "", client.children[j], null);
					    //         links.push({
					    //         	start: i,
					    //         	end: (client.children[j]+randId).replace(/\s+/g,'')
					    //         });
					    //     }
					}
					for(i in status.gr) {
					    client = status.gr[i];
					    addDiv("clients", i, "robot", client.status, "GR", client.ip);

					    if(!!client.children)
					        for(var j=0; j<client.children.length; j++) {
					        	randId = Math.round(Math.random()*1000);
					            addDiv("children", (client.children[j].replace(/\s+/g,''))+randId, "arduino", "", client.children[j], null);
					            links.push({
					            	start: i,
					            	end: (client.children[j]+randId).replace(/\s+/g,'')
					            });
					        }
					}
					for(i in status.pr) {
					    client = status.pr[i];
					    addDiv("clients", i, "robot", client.status, "PR", client.ip);

					    // console.log(client.children.length);
					    // console.log(client.children);
					    if(!!client.children)
					        for(var j=0; j<client.children.length; j++) {
					        	randId = Math.round(Math.random()*1000);
					            addDiv("children", (client.children[j].replace(/\s+/g,''))+randId, "arduino", "", client.children[j], null);
					            links.push({
					            	start: i,
					            	end: (client.children[j]+randId).replace(/\s+/g,'')
					            });
					        }
					}

		        updateArrows();
		    } else
		        printNotConnected();
		};

		function clearColumns () {
		    document.getElementById("webclients").innerHTML = "";
		    document.getElementById("brain").innerHTML = "";
		    document.getElementById("clients").innerHTML = "";
		    document.getElementById("children").innerHTML = "";
		}



	/* --------- Links ------------- */

		function linkDivs (div1Id, div2Id, colId) {
			var div1OffsetTop = $("#"+div1Id).offset().top - $("#"+div1Id).parent().offset().top;
			// console.log("#"+div2Id);
			var div2OffsetTop = $("#"+div2Id).offset().top - $("#"+div2Id).parent().offset().top;
		    var middleDiv1 =  div1OffsetTop + parseFloat($("#"+div1Id).parent().css("margin-top")) + $("#"+div1Id).outerHeight()/2;
		    var middleDiv2 = div2OffsetTop + parseFloat($("#"+div2Id).parent().css("margin-top")) + $("#"+div2Id).outerHeight()/2;
		    var widthCol = $("#"+colId).width();
		    document.getElementById(colId).innerHTML += "\n<path d='M0," + middleDiv1 + " L" + widthCol + "," + middleDiv2 + "' class='link'/>";   
		}

		function linkDivsArc (div1Id, div2Id, colId) {
			// console.log(div1Id + " > " + div2Id);
			var div1OffsetTop = $("#"+div1Id).offset().top - $("#"+div1Id).parent().offset().top;
			var div2OffsetTop = $("#"+div2Id).offset().top - $("#"+div2Id).parent().offset().top;
		    var middleDiv1 =  div1OffsetTop + parseFloat($("#"+div1Id).parent().css("margin-top")) + $("#"+div1Id).outerHeight()/2;
		    var middleDiv2 = div2OffsetTop + parseFloat($("#"+div2Id).parent().css("margin-top")) + $("#"+div2Id).outerHeight()/2;
		    // var middleDiv1 = document.getElementById(div1Id).offsetTop + parseFloat(window.getComputedStyle(document.getElementById(div1Id)).marginTop) + window.getComputedStyle(document.getElementById(div1Id)).height.replace("px", "")/2;
		    // var middleDiv2 = document.getElementById(div2Id).offsetTop + parseFloat(window.getComputedStyle(document.getElementById(div2Id)).marginTop) + window.getComputedStyle(document.getElementById(div2Id)).height.replace("px", "")/2;
		    var dist = middleDiv2 - middleDiv1;
		    // var side = dist>0?1:0;
		    var side = 1;
		    var path = "<path d='M 0," + (middleDiv1<middleDiv2?middleDiv1:middleDiv2) + " a" + Math.abs(dist) + "," + Math.abs(dist) + " 0 0 " + side + " 0," + Math.abs(dist) + "' class='link'/>";
		    document.getElementById(colId).innerHTML += path;
		}

		function clearArrows () {
		    document.getElementById("arrows1").innerHTML = "";
		    document.getElementById("arrows2").innerHTML = "";
		    document.getElementById("arrows3").innerHTML = "";
		}

		function updateArrows (){
		    clearArrows();
		    var source, target;
		    // console.log(links);
		    for (var i = 0; i < document.querySelectorAll(".webclient").length; i++) {
		        source = document.querySelectorAll(".webclient")[i].id;
		        linkDivs(source, "server", "arrows1");
		    }

		    for (i = 0; i < document.querySelectorAll(".client").length; i++) {
		        target = document.querySelectorAll(".client")[i].id;
		        linkDivs("server", target, "arrows2");
		    }

		    for (i = 0; i < document.querySelectorAll(".brain").length; i++) {
		        source = document.querySelectorAll(".brain")[i].id;
		        linkDivsArc(source, "server", "arrows2");
		    }

		    for (i = 0; i < links.length; i++) {
		        linkDivs(links[i].start, links[i].end, "arrows3");
		    }
		}


	// Edit span content
		// Adds the possibility to edit a span content (uncomment updateIpServer too)
		// function addServerEvents(){
		//     $(document).one("click", "#server span.ip", function(e) {
		//         $(this).html("<input type='text' id='ipServer' style='width:70%' value='"+$(this).html()+"'></input><button id='updateIpServer'>✓</button>");

		//         $("#ipServer").select();

		//         $(document).one("click", "#updateIpServer", function(e) {
		//             updateIpServer($("#ipServer").val());

		//             addServerEvents();
		//         });
		//     });
		// }

	this.init = function (){
		Client.order(function (from, name, params){
			if (name == "reseau"){
			// console.log("[Network log] Network updated");
				this.network = params.network;

				if ($rootScope.act_page == 'reseau') {
				    this.updateLayout(this.network);
					$rootScope.$apply();
				}
			}
		}.bind(this));


		window.onresize = function () {
			if ($rootScope.act_page == 'reseau') {
		    	this.updateLayout(this.network);
			}
		}.bind(this);

		// setInterval(function () {
		//     this.updateLayout(this.network);
		// }.bind(this), 1000);
	};

}]);