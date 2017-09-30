"use strict";

class RobotDisplay {
    constructor (name, client) {
        this.name = name;
		this.client = client;

        // Default for all robots
        this.pwm_left = 50;
        this.pwm_right = 50;
        this.pwm_ms = 1000;
        this.speed_l = 1000;
        this.speed_a = 0;
        this.speed_ms = 1000;
        this.a = 0;
        this.x = 0;
        this.y = 0;
        this.direction = "forward";
        this.set_x = 0;
        this.set_y = 0;
        this.set_a = 0;
        this.v = 1000;
        this.r = 0.6;
        this.PID_P = 0.25;
        this.PID_I = 130;
        this.PID_D = 0;
        this.acc = 1500;
		this.inCollision = false;
    }



    // General
	clean () {
		this.client.send(this.name, "clean");
	}

	stop () {
		this.client.send(this.name, "stop");
	}

    // Asserv
    setPWM () {
		this.client.send(this.name, "asserv.pwm", {
			left: this.pwm_left,
			right: this.pwm_right,
			ms: this.pwm_ms
		});
	}

	goPos () {
		this.client.send(this.name, "asserv.goxy", {
			x: parseInt(this.x),
			y: parseInt(this.y),
            direction : this.direction
		});
	}

	goAngle () {
		this.client.send(this.name, "asserv.goa", {
			a: parseFloat(this.a)*Math.PI/180
		});
	}

	goPosAngle () {
		this.goPos();
		this.goAngle();
	}

    goSpeed () {
        this.client.send(this.name, "asserv.speed", {
            l: this.speed_l,
            a: this.speed_a,
            ms: this.speed_ms
        });
    }

	setVit () {
		this.client.send(this.name, "asserv.setvit",{
				v: parseInt(this.v),
				r: parseFloat(this.r)
		});
	}

	setAcc () {
		this.client.send(this.name, "asserv.setacc", {
			acc: parseInt(this.acc)
		});
	}

	setPos () {
		this.client.send(this.name, "asserv.setpos", {
			x: parseInt(this.set_x),
			y: parseInt(this.set_y), 
			a: parseFloat(this.set_a)*Math.PI/180
		});
	}

	setPID () {
		this.client.send(this.name, "asserv.setpid", {
			p: parseFloat(this.PID_P),
			i: parseFloat(this.PID_I), 
			d: parseFloat(this.PID_D)
		});
	}

	toggleCollision () {
		this.inCollision = !this.inCollision;
		this.client.send (this.name, "asserv.collision", {
			activate: this.inCollision
		});
	}
}