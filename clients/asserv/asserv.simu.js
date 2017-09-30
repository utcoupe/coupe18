/**
 * Module Asservissement en mode simulateur
 *
 * @module clients/Asserv/AsservSimu
 * @requires module:clients/Asserv/Asserv
 */

"use strict";

const Asserv = require('./asserv');

/**
 * Classe héritant de l'asservissement, mode simulateur
 *
 * @memberof module:clients/Asserv/AsservSimu
 * @extends {clients/Asserv/Asserv.Asserv}
 */
class AsservSimu extends Asserv{
	constructor(robot){
		super(robot);
		this.SIMU_FACTOR_VIT = 1;
		this.SIMU_FACTOR_A = 0.3;
		this.FPS = 30;
		this.timeouts = [];
		this.pos = {x:0,y:0,a:0};
		this.speed = 300; // 800;
        // Not used in simu
        this.acc = 0;
	}

	stop() {
		this.clean();
	}

    simuDistRotation(angle) {
        return Math.abs(angle)*100; // Approximate radius of 10cm
    }

    simuRotationTime(angle, speed) {
        return this.simuDistRotation(angle)/(speed*this.SIMU_FACTOR_VIT*this.SIMU_FACTOR_A);
    }

    simuDist(pwm, dt, speed) {
        return (speed*this.SIMU_FACTOR_VIT)*dt;
    }

	/**
	 * Sets Position
	 *
	 * @param {Object} pos
	 */
	setPos(pos) {
        this.pos.x = pos.x;
        this.pos.y = pos.y;
        this.setA(pos.a);
		this.sendPos();
        this.fifo.orderFinished();
	}

	/**
	 * Clean
	 */
	clean(){
		//this.logger.debug('cleaning %d this.timeouts', this.timeouts.length);
		while(this.timeouts.length > 0) {
			clearTimeout(this.timeouts.shift());
		}
	}

	/**
	 * Set speed
	 *
	 * @param {int} v Speed
	 * @param {int} r rotation
	 */
    setSpeed(v, r) {
		this.speed = parseInt(v);
        this.fifo.orderFinished();
	}

    setAcc(acc) {
        this.acc = parseInt(acc);
        this.fifo.orderFinished();
    }

	/**
	 * Calage X
	 *
	 * @param {int} x
	 * @param {int} a Angle
	 */
	calageX(x, a) {
		this.setPos({x: x, y: this.pos.y, a: a});
	}
	/**
	 * Calage Y
	 *
	 * @param {int} y
	 * @param {int} a Angle
	 */
	calageY(y, a) {
		this.setPos({x: this.pos.x, y: y, a: a});
	}

	/**
	 * Speed ?
	 *
	 * @param {int} l
	 * @param {int} a Angle
	 * @param {int} ms
	 */
	speed(l, a, ms) {
        for(var t = 0; t < ms; t += 1000/this.FPS) {
            this.timeouts.push(setTimeout(this.simu_speed(l, this.pos.x, this.pos.y, this.pos.a, t), t));
        }
        this.timeouts.push(setTimeout(this.simu_speed(l, this.pos.x, this.pos.y, this.pos.a, ms), ms));
        this.timeouts.push(setTimeout(function() { this.fifo.orderFinished(); }.bind(this), ms));
	};

	/**
	 * Pulse Width Modulation
	 *
	 * @param {string} left
	 * @param {string} right
	 * @param {int} ms
	 */
	pwm(left, right, ms) {
        var pwm = (left+right)/2;
        for(var t = 0; t < ms; t += 1000/this.FPS) {
            this.timeouts.push(setTimeout(this.simu_pwm(pwm, this.pos.x, this.pos.y, this.pos.a, t), t));
        }
        this.timeouts.push(setTimeout(this.simu_pwm(pwm, this.pos.x, this.pos.y, this.pos.a, ms), ms));
        this.timeouts.push(setTimeout(function() { this.fifo.orderFinished(); }.bind(this), ms));
	}

	/**
	 * Go X Y
	 *
	 * @param {int} x
	 * @param {int} y
	 * @param {string} direction
	 */
	goxy(x, y, direction) {
        var dx = x-this.pos.x;
        var dy = y-this.pos.y;
        var dist = Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2));
        var tf = (dist / (this.speed*this.SIMU_FACTOR_VIT))*1000; // *1000 s->ms

        var angle_front = this.convertA(Math.atan2(dy,dx)-this.pos.a);
        var angle_left = this.convertA(angle_front+Math.PI);
        var angle_init;

        //todo english names
        if(direction == "forward") angle_init = angle_front;
        else if(direction == "backward") angle_init = angle_left;
        else if (Math.abs(angle_front) < Math.abs(angle_left)) angle_init = angle_front;
        else angle_init = angle_left;

        // logger.debug("dx: ", dx);
        // logger.debug("dy: ", dy);
        // logger.debug("angle: ", this.pos.a);
        // logger.debug("angle avant: ", angle_front);
        // logger.debug("angle arriere: ", angle_left);
        // logger.debug("angle depart: ", angle_init);

        this.goa(angle_init+this.pos.a, function() {
            for(var t = 0; t < tf; t += 1000/this.FPS) {
                this.timeouts.push(setTimeout(this.simu_goxy(this.pos.x+dx*t/tf, this.pos.y+dy*t/tf), t));
            }
            this.timeouts.push(setTimeout(this.simu_goxy(x, y), tf));
            this.timeouts.push(setTimeout(function() { this.fifo.orderFinished(); }.bind(this), tf));
        }.bind(this));
	}

	/**
	 * Simu Go Angle
	 *
	 * @param {int} a Angle
	 */
	goa(a, callback){
		a = this.convertA(a);
		var da = this.convertA(a-this.pos.a);
		// logger.debug("depart:", this.pos.a);
		// logger.debug("arrivee:", a);
		// logger.debug("delta:", da);

		var tf = this.simuRotationTime(da, this.speed)*1000; // *1000 s->ms
        for(var t = 0; t < tf; t += 1000/this.FPS) {
            this.timeouts.push(setTimeout(this.simu_goa(this.pos.a+da*t/tf), t));
        }
        this.timeouts.push(setTimeout(this.simu_goa(a), tf));
        if (callback !== undefined) {
            this.timeouts.push(setTimeout(() => { callback.call(this); }, tf)); // arrow function to simply bind this
        } else {
            this.timeouts.push(setTimeout(function() { this.fifo.orderFinished(); }.bind(this), tf));
        }
	}

	/**
	 * Set P I D
	 *
	 * @param {int} p
	 * @param {int} i
	 * @param {int} d
	 */
	setPid(p, i, d){
        // Nothing to do, because pid in simu will no be taken in account
        this.fifo.orderFinished();
	}

	doStartSequence(params){
        this.fifo.orderFinished();
	}

    setEmergencyStop (activate) {
        this.logger.info("Received emergency in asserv : " + activate);
        super.setEmergencyStop(activate);
    }

    pause () {
        this.logger.info("Received a pause order");
    }

    resume () {
        this.logger.info("Received a resume order");
    }

    simu_speed(speed, x, y, a, dt) {
        return function() {
            this.pos = {
                x: x + Math.cos(a) * speed*dt/1000,
                y: y + Math.sin(a) * speed*dt/1000,
                a: a
            };
            this.sendPos();
        }.bind(this);
    }

    simu_goa(a) {
        return function() {
            this.setA(a);
            this.sendPos();
        }.bind(this);
    }

    simu_goxy(x, y) {
        return function() {
            this.pos.x = x;
            this.pos.y = y;
            this.sendPos();
        }.bind(this);
    }

    simu_pwm(pwm, x, y, a, dt) {
        return function() {
            this.pos = {
                x: x + Math.cos(a) * this.simuDist(pwm, dt/1000, this.speed),
                y: y + Math.sin(a) * this.simuDist(pwm, dt/1000, this.speed),
                a: a
            };
            this.sendPos();
        }.bind(this);
    }
}

// Exports an object to be sure to have a single instance in the system
module.exports = function(robot) {
    return new AsservSimu(robot);
};
