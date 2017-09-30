"use strict";

class GrobotDisplay extends RobotDisplay {
    constructor (name, client) {
        super(name, client);

        this.PID_P = 0.5;
        this.PID_I = 0;
        this.PID_D = 0;

        // Specific to grobot
        // example : this.PID_P = 0.5
    }

    openTrunk () {
        this.client.send("canon", "open_trunk");
    }

    closeTrunk () {
        this.client.send("canon", "close_trunk");
    }

    throwBalls () {
        this.client.send("canon", "throw_balls");
    }

    turnOnCanon () {
        this.client.send("canon", "turn_on");
    }

    turnOffCanon () {
        this.client.send("canon", "turn_off");
    }

    turnOnSweeper () {
        this.client.send("sweeper", "turn_on");
    }

    turnOffSweeper () {
        this.client.send("sweeper", "turn_off");
    }

    funnyAction () {
        this.client.send("rocket", "funny_action");
    }

    seesaw() {
        this.client.send(this.name, "climb_seesaw");
    }

    swallowBalls() {
        this.client.send("sweeper", "swallow_balls");
    }
}