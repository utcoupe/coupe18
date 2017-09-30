"use strict";

class TibotDisplay extends RobotDisplay {
    constructor (name, client) {
        super(name, client);

        // Specific to tibot
        // example : this.PID_P = 0.5
        this.moduleColor = "null";
        this.pushTowards = "dont";
        this.nbModules = 0;
    }

    // Actuators
    closeGrabber () {
		this.client.send("unit_grabber", "closeGrabber");
	}

	openGrabber () {
        this.client.send("unit_grabber", "openGrabber");
	}

	openArms () {
        this.client.send("unit_grabber", "openArm");
	}

	closeArms () {
        this.client.send("unit_grabber", "closeArm");
	}

    startRotate () {
        this.client.send("unit_grabber", "startArmRotate");
    }

    stopRotate () {
        this.client.send("unit_grabber", "stopArmRotate");
    }

    takeModule () {
        this.client.send("unit_grabber", "take_module");
    }

	drop () {
		this.client.send("base_constructor", "drop");
	}

    engageModule () {
        this.client.send("base_constructor", "engage");
    }

    rotateModule () {
        this.client.send("base_constructor", "rotate", {color: this.moduleColor});
    }

    pushModule () {
        this.client.send("base_constructor", "push", {push_towards: this.pushTowards});
        this.logger.error("ok");
    }

    prepareModule () {
        this.client.send("base_constructor", "prepare_module", {
            color: this.moduleColor,
            push_towards: this.pushTowards
        });
    }

    dropModule () {
        this.client.send("base_constructor", "drop_module", {
            nb_modules_to_drop: this.nbModules
        });
    }
}