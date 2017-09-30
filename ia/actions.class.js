"use strict";

var log4js = require('log4js');

/**
 * Actions Module
 * 
 * @module ia/actions
 * @requires log4js
 * @see {@link ia/actions.Actions}
 */

class Actions{
	/**
	 * Actions Constructor
	 * 
	 * @exports ia/actions.Actions
	 * @constructor
	 * @param ia
	 */
	constructor(ia, robot) {

		// this.__dist_startpoints_plot = 120;
		// this.__nb_startpoints_plot = 16;

		/** IA */
		this.ia = ia;

		/** Robot we are affected to */
		this.robot = robot;

		this.logger = log4js.getLogger('ia.' + this.robot.name + '.actions');

		/** Tasks to do*/
		this.todo = {};
		/** Tasks in progress */
		this.inprogress = null;
		/** Done */
		this.done = {};
		/** killed */
		this.killed = {};

		/** Valid id do action */
		this.valid_id_do_action = -1;

		this.todo = this.importActions(ia.data);
	}

	// /**
	//  * Convert Angle
	//  * 
	//  * @param {int} a Angle
	//  */
	// function convertA(a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

	/**
	 * Collision
	 */
	collision () {
		if(this.inprogress !== null) {
			this.todo[this.inprogress.name] = this.inprogress;
			this.inprogress = null;
		}
	}

	/**
	 * Import Actions from data
	 * 
	 * @param {Object} data
	 */
	importActions(data) {
		var req;

		try {
			req = require('./actions.json');
		}
		catch(err) {
		    this.logger.fatal("Erreur lors de l'importation des actions dans l'IA: "+err);
		}
		var actions = {};

		// Delete actions that doesn't belong to this robot
		for (let action_name in req.actions){
			if (req.actions[action_name].owner == this.robot.name) {
				// logger.debug(actions[action_name].owner + " != " + this.robot.name);
				actions[action_name] = req.actions[action_name];
			}
		}
		this.logger.info(Object.keys(actions).length + " actions imported");

		// Link "object" with exiting thing in the Data class
		Object.keys(actions).forEach(function(i) {
			actions[i].object = data.getObjectRef(actions[i].objectname);
			actions[i].name = i;

			// Delete action if it's not for us
			if (actions[i].object.color != this.ia.color
				&& actions[i].object.color != "both") {
				delete actions[i];
			} else {

				// Automatically create startpoints for objects that need it
				// if ((actions[i].object !== null) && (actions[i].type == "plot") && (actions[i].startpoints.length === 0)) {
				// 	actions[i].startpoints.push({
				// 		x: actions[i].object.pos.x,
				// 		y: actions[i].object.pos.y
				// 	});
				// 	// var temp;
				// 	// for(var j = 0; j < __nb_startpoints_plot; j++) {
				// 	// 	temp = j*2*Math.PI/__nb_startpoints_plot;
				// 	// 	actions[i].startpoints.push({
				// 	// 		x: actions[i].object.pos.x + __dist_startpoints_plot * Math.cos(temp),
				// 	// 		y: actions[i].object.pos.y + __dist_startpoints_plot * Math.sin(temp),
				// 	// 		a: convertA(temp+Math.PI)
				// 	// 	});
				// 	// }
				// }
				// else if((actions[i].object !== null) && (actions[i].type == "clap")) {
				// 	if(this.ia.color != "blue") {
				// 		var a = actions[i].startpoints[0].a;
				// 		actions[i].startpoints[0].a = (a < 0) ? -Math.PI - a : Math.PI - a;
				// 	}
				// }

			}
		}.bind(this));

		return actions;
	};

	/**
	 * Parse Order
	 * 
	 * @param {string} from Sender
	 * @param {string} name Name of the action
	 * @param {Object} params Parameters of the action
	 */
	parseOrder (from, name, params) {
		switch(name) {
			case 'action_finished':
			// this.logger.debug('received action_finished');
				this.actionFinished();
			break;
			case 'path_finished':
				// this.logger.debug('received path_finished');
				this.pathFinished();
			break;
			default:
				this.logger.warn('Ordre inconnu dans ia.' + this.robot.name + '.actions: '+name);
		}
	};


	/**
	 * Kill an action
	 * 
	 * @param {string} action_name
	 */
	kill (action_name){
		this.logger.debug("kill: verify this function");
		// If action doesn't exist
		if (!!action_name && this.exists(action_name)){
			this.killed[action_name] = this.todo[action_name];
			delete this.todo[action_name];
		}
	};

	/**
	 * Return true if the action exists
	 * 
	 * @param {string} action_name
	 */
	exists (action_name){
		if (!this.todo[action_name]){
			if (!this.killed[action_name] && (this.inprogress.name != action_name) && /*!this.inprogress[action_name] &&*/ !this.done[action_name])
				this.logger.warn("Action named '"+ action_name +"' doesn't exist");
			else
				this.logger.warn("Action named '"+ action_name +"' already killed in progress or done !");
			return false;
		} else {
			return true;
		}
	};

	/**
	 * Return true if the action is done
	 * 
	 * @param {string} action_name
	 */
	isDone (action_name){
		return !action_name || this.done.hasOwnProperty(action_name);
	};

	/**
	 * Return the distance between two points
	 * 
	 * @param {Object} A
	 * @param {int} A.x
	 * @param {int} A.y
	 * @param {Object} B.x
	 * @param {object} B.y
	 */
	norm2Points(A, B) {
		return Math.sqrt(Math.pow(A.x-B.x, 2) + Math.pow(A.y-B.y, 2));
	}
	/**
	 * Return the distance between a position and the position of the current action
	 * 
	 * @param {Object} pos
	 * @param {int} pos.x
	 * @param {int} pos.y
	 * @param {string} an Action name
	 */
	getNormAction(pos, an) {
		return this.norm2Points(pos, this.todo[an].object.pos);
	};
	
	/**
	 * Get priority Action
	 * 
	 * @param {string} an Action name
	 */
	getPriorityAction(an) {
		return this.todo[an].object.status == "lost" ? (this.todo[an].priority - 100) : this.todo[an].priority;
	};
	
	/**
	 * Return true id is doable
	 * 
	 * @param {Object} action
	 */
	isDoable(action) {
		// Verifies some things about the action

		if (!!action.dependency && !this.isDone(action.dependency)){
			// Depends on an action, but it hasn't already been done
			return false;
		}

		if (action.dependencyRobotContent !== undefined){
			// Depends on the robot content

			// if ((action.dependencyRobotContent.gobelet !== undefined) &&
			// 	(this.robot.content.gobelet !== action.dependencyRobotContent.gobelet)){
			// 	// The cup holder position isn't consistent with needed state
			// 	return false;
			// }

			// If there's a constraint about the current number of modules
			if ((action.dependencyRobotContent.supModule  !== undefined)
				&& (this.robot.content.nb_modules  !== undefined)
				&& (this.robot.content.nb_modules > action.dependencyRobotContent.supModule)){
				// More than the supremum
				return false;
			}
			if ((action.dependencyRobotContent.infModule  !== undefined)
				&& (this.robot.content.nb_modules  !== undefined)
				&& (this.robot.content.nb_modules < action.dependencyRobotContent.infModule)){
				// Less than the infimum
				return false;
			}
		 
		}

		// Lost objects are actually done in priority
		// if (action.object.status == "lost"){
		// 	return false;
		// }

		return true;
	};
	
	/**
	 * Do next Action
	 * 
	 * @param callback
	 */
	doNextAction(callback, otherRobotPos) {
		this.valid_id_do_action++;
		var actions_todo = [];

		// Get les actions possibles
		Object.getOwnPropertyNames(this.todo).forEach(function(an) { //an = action name
			if(this.isDoable(this.todo[an])) {
				actions_todo.push(an);
			}
		}, this);

		// Tri par priorité puis par norme
		var pos = this.robot.pos;
		actions_todo.sort(function(a, b) {
			return (this.getPriorityAction(b) - this.getPriorityAction(a)) || (this.getNormAction(pos, a) - this.getNormAction(pos, b));
		}.bind(this));

		// // Print possible actions
		// for(var i in actions_todo) {
		// 	this.logger.debug('[%d] %s (%d)', this.todo[actions_todo[i]].priority, actions_todo[i], parseInt(this.getNormAction(pos, actions_todo[i])));
		// }

		// Va choisir l'action la plus proche, demander le path et faire doAction
		this.pathDoAction(callback, actions_todo, this.valid_id_do_action, otherRobotPos);
	};

	/**
	 * Return nearest Startpoint
	 * 
	 * @param {Object} pos
	 * @param {int} pos.x
	 * @param {int} pos.y
	 * @param {Object} startpoints
	 */
	getNearestStartpoint(pos, startpoints) {
		var min_dist = Infinity;
		var nearest = null;

		for (var i = 0; i < startpoints.length; i++) {
			var dist = this.norm2Points(pos, startpoints[i]);

			if (dist < min_dist){
				min_dist = dist;
				nearest = startpoints[i];
			}
		}

		return nearest;
	};

	/**
	 * Find the path to the action
	 * 
	 * @param callback
	 * @param {Object} actions
	 * @param {int} id
	 */
	pathDoAction(callback, actions, id, otherRobotPos) {
		if(id != this.valid_id_do_action) {
			this.logger.debug('id different');
			return;
		}
		// Va choisir l'action la plus proche, demander le path et faire doAction
		if(actions.length > 0) {
			var actionName = actions.shift();
			var action = this.todo[actionName];
			var startpoint = this.getNearestStartpoint(this.robot.pos, action.startpoints);
			if (startpoint === null
				&& action.type == "module") {
				this.logger.warn("TMP startpoint for module");
				startpoint = {
					x: action.object.pos.x,
					y: action.object.pos.y,
					a: 0
				}
			}
			var objectpoint = {
				x: action.object.pos.x,
				y: action.object.pos.y,
				a:0
			}
			this.logger.debug("Asking path to " + actionName + ' at [' + startpoint.x + ", " + startpoint.y + ']');
			this.ia.pathfinding.getPath(this.robot.pos, startpoint, this.robot, objectpoint, function(path) {
				if (this.robot.paused) {
					this.logger.warn("Path returned but robot has been paused meanwhile, abort");
					return;
				}

				if(path !== null) {
					this.robot.path = path;
					this.doAction(callback, action, startpoint, id);
				} else {
					this.logger.debug("Path to " + actionName + " not found");
					// Si le pathfinding foire, on fait la deuxième action la plus importante
					this.pathDoAction(callback, actions, id, otherRobotPos);
				}
			}.bind(this));
		} else {
			this.logger.debug("No valid path were found, trying again in 0.5s");
			setTimeout(function() {
				this.doNextAction(callback, otherRobotPos);
			}.bind(this), 500);
		}
	};

	/**
	 * Do Action
	 * 
	 * @param callback
	 * @param {Object} action
	 * @param {Object} startpoint
	 * @param {int} startpoint.x
	 * @param {int} startpoint.y
	 * @param {int} id
	 */
	doAction (callback, action, startpoint, id) {
		if(id != this.valid_id_do_action)
			return;
		this.callback = callback;
		
		// // Change action to state "in progress"
		this.inprogress = action;
		delete this.todo[action.name];

		this.logger.info('Current action : %s (%d;%d;%d)', action.name, startpoint.x, startpoint.y, startpoint.a);
		this.robot.path.map(function(checkpoint) {
			this.ia.client.send(this.robot.name, "asserv.goxy", {
				x: checkpoint.x,
				y: checkpoint.y,
				direction: action.direction
			});
		}, this);
		if(!!startpoint.a) {
			this.ia.client.send(this.robot.name, "asserv.goa", {
				a: startpoint.a
			});
		}

		if (!!action.preparation_orders) {

			action.preparation_orders.forEach(function (order, index, array){
				if (!!this.robot.content.modules_color
					&& order.params.color == "TO_BE_FILLED_BY_IA") {
					// this.logger.warn("TODO: drop color depending on robot content");
					// if (this.robot.content.modules_color == this.ia.color) {
					// 	// If all the modules are of our color, it doesn't matter how we drop them
					// 	order.params.color = "null";
					// } else {
					// 	// Some modules are multicolor, we must rotate the modules
					// 	order.params.color = this.ia.color;
					// }
					order.params.color = this.ia.color;
				}
				let dest = !!order.dest ? order.dest : this.robot.name;
				this.ia.client.send(dest, order.name, order.params);
			}.bind(this));
		}


		this.ia.client.send(this.robot.name, "asserv.send_message", {
			name: this.robot.name + ".actions.path_finished"
		});

		// // Change action and its "to be killed" actions to state done

		// console.log(this.todo);
		// console.log(this.inprogress);
		// console.log(this.done);
	};

	/**
	 * Action Finished
	 */
	actionFinished () {
		if(this.inprogress !== null) {
			this.done[this.inprogress.name] = this.inprogress;
			this.logger.info('Action %s est finie !', this.inprogress.name);
			this.inprogress.object.status = "taken";
			this.inprogress = null;
			var temp = this.callback;
			this.callback = function() {this.logger.warn('callback vide'); };
			temp();
		}
	}


	/**
	 * Do action once arrived
	 */
	pathFinished () {
		this.robot.path = [];

		// 1 order for 1 action -> not any more
		let dest = this.robot.name;
        if (this.inprogress != null) {
            this.inprogress.orders.forEach(function (order, index, array){
                // this.ia.client.send(this.robot.name, action.orders[0].name, action.orders[0].params);
                dest = !!order.dest ? order.dest : this.robot.name;
                this.ia.client.send(dest, order.name, order.params);
            }.bind(this));

            this.ia.client.send(dest, "send_message", {
                name: this.robot.name + ".actions.action_finished",
                action_name: this.inprogress.name
            });
        }
	}

	/**
	 * Return the distance between two positions
	 * 
	 * @param {Object} pos1
	 * @param {int} pos1.x
	 * @param {int} pos1.y
	 * @param {Object} pos2
	 * @param {int} pos2.x
	 * @param {int} pos2.y
	 */
	getDistance (pos1, pos2){
		return Math.sqrt(Math.pow(pos1.x - pos2.x, 2) + Math.pow(pos1.y - pos2.y, 2));
	};

	/**
	 * Takes a position and the ennemy robot # to put everything in its surroundings (~ 1.1 * radius) as "lost"
	 * the enemy went there, don't do that !
	 * 
	 * @param {Object} pos
	 * @param {int} e_robot_id
	 */
	killObjects (dots) {
		// this.logger.debug("TODO: kill objects/actions by which the enemy went");
		Object.keys(this.todo).forEach(function(a) {
			if ((this.todo[a].object.color == "both")
				&& ((this.getDistance(this.ia.data.erobot[0].pos, this.todo[a].object.pos) < 0.55*this.ia.data.erobot[0].d)
					|| (this.getDistance(this.ia.data.erobot[1].pos, this.todo[a].object.pos) < 0.55*this.ia.data.erobot[1].d))
				&& (this.todo[a].object.status != "lost")) {
				this.logger.info("The object " + this.todo[a].objectname + " linked to the action " + a + " of type  " + this.todo[a].type + " is recorded as lost");
				this.todo[a].object.status = "lost";
			}
		}.bind(this));
	}
	
}

module.exports = Actions;
