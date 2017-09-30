/**
 * @file Fichier permettant de faire le lien entre le webclient et le simulateur
 */
//"use strict"; Utile ?

angular.module('app').controller('SimulateurCtrl', ['$rootScope', '$scope', 'Client', 'Simulateur',
	function ($rootScope, $scope, Client, Simulateur) {
		$rootScope.act_page = 'simulateur';
		if (!Simulateur.controllerSimu)
		{
			console.warn('Création d\'un nouveau simulateur !');
			Simulateur.controllerSimu = new Controller("3dobjects.json", "../simulateur/");
			Simulateur.controllerSimu.createRenderer();
			Simulateur.controllerSimu.loadParameters();
		}
		else
			Simulateur.controllerSimu.updateRenderer();
		$scope.pos_pr = new Position();
		$scope.rot_pr = new Position();
		$scope.pos_gr = new Position();
		$scope.rot_gr = new Position();
		$scope.pos_epr = new Position();
		$scope.pos_egr = new Position();
		$scope.vueDeFace = function () { Simulateur.controllerSimu.selectView("front"); }
		$scope.vueDeDessus = function () { Simulateur.controllerSimu.selectView("top"); }
		$scope.vueDeDerriere = function () { Simulateur.controllerSimu.selectView("behind"); }
		$scope.vueDeGauche = function () { Simulateur.controllerSimu.selectView("left"); }
		$scope.vueDeDroite = function () { Simulateur.controllerSimu.selectView("right"); }
		$scope.iaJack = function () { Client.send("ia", "ia.jack"); }
		$scope.iaPlacerPr = function () { Client.send("ia", "pr.placer"); }
		$scope.iaCollisionPr = function () { Client.send("ia", "pr.collision"); }
		$scope.iaStop = function () { Client.send("ia", "ia.stop"); }

		Simulateur.updateInterface = function () {
			$scope.pos_pr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.pr.color).position;
			$scope.rot_pr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.pr.color).rotation;
			$scope.pos_gr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.gr.color).position;
			$scope.rot_gr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.gr.color).rotation;
			$scope.pos_epr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.epr.color).position;
			$scope.pos_egr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.egr.color).position;
		}
	}]);

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

		if(!!data_robot.path && data_robot.path.length > 1){
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
function updatePath(path, simulateur, robot)
{
	var PATH_HIGHT = 0.1;
	simulateur.controllerSimu.objects3d.get(robot).showPath(
		path.map( (pos) => {
			act_pos = convertPosNew({x: pos[0], y: pos[1]});
			return new Position(act_pos.x, PATH_HIGHT, act_pos.z);
		})
	);
}

/**
 * Supprime le chemin du robot
 * 
 * @param {Array<Number>} path 
 * @param {Object} simulateur 
 * @param {String} robot 
 */
function clearPath(simulateur, robot)
{
	simulateur.controllerSimu.objects3d.get(robot).clearPath();
}

angular.module('app').service('Simulateur', ['$rootScope', 'Client', function ($rootScope, Client) {
	this.init = function () {
		Client.order(function (from, name, data) {
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
		}.bind(this));
	};
}]);
