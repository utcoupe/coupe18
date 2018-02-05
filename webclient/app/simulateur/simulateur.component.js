/* eslint-disable no-undef */

class SimulateurController {
	constructor ($rootScope, $scope, Ros, Simulateur) {
		$rootScope.act_page = 'simulateur';
		if (!Simulateur.controllerSimu)
		{
			console.warn('Création d\'un nouveau simulateur !');
			Simulateur.controllerSimu = new Controller("3dobjects.json", "app/simulateur/simulateur/");
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
		// $scope.iaJack = function () { Client.send("ia", "ia.jack"); }
		// $scope.iaPlacerPr = function () { Client.send("ia", "pr.placer"); }    //FIXME
		// $scope.iaCollisionPr = function () { Client.send("ia", "pr.collision"); }
		// $scope.iaStop = function () { Client.send("ia", "ia.stop"); }

		Simulateur.updateInterface = function () {
			$scope.pos_pr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.pr.color).position;
			$scope.rot_pr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.pr.color).rotation;
			$scope.pos_gr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.gr.color).position;
			$scope.rot_gr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.gr.color).rotation;
			$scope.pos_epr = Simulateur.controllerSimu.objects3d.get("pr_" + this.robots.epr.color).position;
			$scope.pos_egr = Simulateur.controllerSimu.objects3d.get("gr_" + this.robots.egr.color).position;
		}
	}
}

angular.module('roscc').component('ccSimulateur', {
  templateUrl: 'app/simulateur/simulateur.html',
  controller: SimulateurController
});

/* eslint-enable no-undef */
