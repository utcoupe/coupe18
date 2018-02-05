/* eslint-disable */

//FIXME : à mettre dans le service
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



angular.module('roscc').service('Simulateur', ['$rootScope', 'Ros',
function () {
	this.init = function () {
		/*Client.order(function (from, name, data) {
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
		}.bind(this));*/ //FIXME
	};
}]);

/* eslint-enable */
