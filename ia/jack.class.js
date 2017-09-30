/**
 * Jack Module
 * Sends "jack" message when detected on GPIO
 * 
 * @module ia/jack
 * @requires log4js
 * @see {@link ia/jack.Jack}
 */

const gpio = require('rpi-gpio');
const PLUS = 13; 	// Switch pin 3, OUTPUT +3.3V
const MINUS = 11; 	// Switch pin 1, INPUT
const LED = 7;		// LED pin, OUTPUT +3.3V
const logger = require('log4js').getLogger('ia.jack');

module.exports = (function () {
	"use strict";

	/**
	 * Jack Constructor
	 * 
	 * @exports ia/jack.Jack
	 * @constructor
	 * @param {Object} ia
	 */
	function Jack(ia) {
		/** IA */
		this.ia = ia;
		this.prevVal = false;
		this.ledReady = false;
		this.setup();
	}
	/**
	 * Setup Jack
	 */
	Jack.prototype.setup = function() {
		gpio.setup(PLUS, gpio.DIR_OUT, function() {
			gpio.write(PLUS, true, function(err) {
			    if (err) throw err;
			    // logger.info("Jack cocked between " + PLUS + " and " + MINUS);
			});
		});

		gpio.setup(LED, gpio.DIR_OUT, function() {
                        gpio.write(LED, this.prevVal, function(err) {
                            if (err) throw err;
                            // logger.info("Lit up jack LED on " + LED);
				this.ledReady = true;
                        }.bind(this));
                }.bind(this));

		gpio.setup(MINUS, gpio.DIR_IN, () => {
			gpio.read(MINUS, function(err, value) {
				if (value) {
					// In case the jack is already cocked when the AI starts
					this.cockJack();
				}
				this.updateLight(value);
				gpio.setup(MINUS, gpio.DIR_IN, gpio.EDGE_BOTH);
	    		}.bind(this));
		});

		gpio.on('change', (c, v) => {
			this.valueChanged(c, v);
		});
	};

	/**
	 * Read on GPIO, react accordingly
	 */
	Jack.prototype.valueChanged = function(channel, value) {
		// logger.debug("Channel " + channel + " -> " + value);
		if(channel == MINUS && value && !this.prevVal) {
			this.cockJack();
		} else if (channel == MINUS && !value && this.prevVal) {
			logger.info("JAAAAAACCCKK");
			this.ia.client.send("ia", "ia.jack", {});
			this.prevVal = value;
		}
		this.updateLight(value);
	};

	Jack.prototype.cockJack = function() {
		logger.info("Cocked !");
		this.prevVal = true;
	}

	Jack.prototype.updateLight = function(value) {
		if(this.ledReady) {
                        gpio.write(LED, value, function(err) {
                            if (err) throw err;
                        }.bind(this));
                }
	}

	/**
	 * Read on GPIO, react accordingly
	 */
	Jack.prototype.stop = function() {    
		gpio.destroy(function() {
	        	logger.info('All pins unexported');
		});
	};


	return Jack;
})();

