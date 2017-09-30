/**
 * Module Asservissement en mode réel
 *
 * @module clients/Asserv/AsservSimu
 * @requires module:clients/Asserv/Asserv
 */

"use strict";

// the asserv orientation is direct, but the node one is indirect (like the map of the coupe)
// so the inversion is done in this file, for each data send and received

const Asserv = require('./asserv');
const defineParser = require('../shared/defineparser');
const SerialPort = require('serialport');

class AsservReal extends Asserv{
    constructor(robot, serialPort){
        super(robot);
        this.pos = {x:0,y:0,a:0};
        this.asservCommands = defineParser(process.env.UTCOUPE_WORKSPACE + "/arduino/common/asserv/protocol.h");
        this.ordersSerial = undefined;
        // Connected means that the node has started the device through serial port
        this.serialPortConnected = false;
        this.logger.debug("Serial port : " + serialPort);
        this.serialPort = new SerialPort(serialPort, {
            baudrate: 57600,
            parser : SerialPort.parsers.readline("\n")
        });
        this.serialPort.on("data", function(data){
            this.parseCommand(data.toString());
        }.bind(this));
        this.serialPort.on("error", function(data){
            this.logger.error("Serial port error : " + data.toString());
        }.bind(this));
        this.serialPort.on("close", function(){
            this.serialPortConnected = false;
            //todo
            // this.sendStatus();
            this.logger.debug("Serial port close");
        }.bind(this));
    }

    parseCommand(receivedCommand) {
        if (!this.serialPortConnected) {
            //todo robot+"_others"
            // If not connected, wait the ID of the arduino before doing something else
            if (receivedCommand.indexOf(this.robotName + "_asserv") == 0) {
                //todo find a way to make it proper
                var order = [this.asservCommands.START,0].join(";")+";\n";
                this.logger.debug(order);
                this.serialPort.write(order);
            } else {
                this.logger.debug(receivedCommand.toString());
                // Trigger on reception of ack from arduino that it has started
                if (receivedCommand.indexOf("0;") == 0) {
                    this.serialPortConnected = true;
                    this.ordersSerial = require("../shared/orders.serial")(this.serialPort, function(x, y, a) {
                        a /= 1000;
                        let pos = this.robot.posArduinoToIa(x, y, a);
                        if(pos.a >= 0){
                            while(pos.a > Math.PI)
                                pos.a -= 2.0*Math.PI;
                        } else {
                            while(pos.a <= -Math.PI)
                                pos.a += 2.0*Math.PI;
                        }
                        this.logger.debug("New pos : x=" + pos.x + ", y=" + pos.y + ", a=" + pos.a);
                        this.robot.client.send("ia", this.robotName + ".pos", {x: pos.x, y: pos.y, a: pos.a});
                    }.bind(this));
                }
            }
        }
    }

    stop() {
        if (this.serialPort.isOpen()) {
            this.ordersSerial.sendOrder(this.asservCommands.HALT, function() {
                this.serialPort.close();
                this.logger.info("Asserv real has stopped");
            }.bind(this));
        }
        this.serialPortConnected = false;
    }

    /********************************************************************\
     *
     *  ASSERV ABSTRACT FUNCTIONS
     *
     /********************************************************************/

    setPos(pos) {
        if (!!this.ordersSerial) {
            var posToArduino = this.robot.posIaToArduino({
                x: parseInt(pos.x),
                y: parseInt(pos.y),
                a: this.myWriteFloat(pos.a)
            });
            this.ordersSerial.sendOrder(this.asservCommands.SET_POS, [posToArduino.x, posToArduino.y, posToArduino.a], function() { this.fifo.orderFinished(); }.bind(this));
        } else {
            this.fifo.orderFinished();
        }
    }

    clean(){
        //this.logger.debug('cleaning %d this.timeouts', this.timeouts.length);
        this.ordersSerial.sendOrder(this.asservCommands.CLEANG, [], function() { this.fifo.orderFinished(); }.bind(this));
    }

    setSpeed(v, r) {
        this.ordersSerial.sendOrder(this.asservCommands.SPDMAX, [parseInt(v), this.myWriteFloat(r)], function() { this.fifo.orderFinished(); }.bind(this));
    }

    calageX(x, a) {
        this.setPos({x: x, y: this.pos.y, a: a});
    }

    calageY(y, a) {
        this.setPos({x: this.pos.x, y: y, a: a});
    }

    speed(l, a, ms) {
        this.ordersSerial.sendOrder(this.asservCommands.SPD, [parseInt(l), parseInt(a), parseInt(ms)], function() { this.fifo.orderFinished(); }.bind(this));
    };

    setAcc(acc) {
        // this.logger.debug(myWriteFloat(r));
        this.ordersSerial.sendOrder(this.asservCommands.ACCMAX, [parseInt(acc)], function() { this.fifo.orderFinished(); }.bind(this));
    }


    pwm(left, right, ms) {
        this.ordersSerial.sendOrder(this.asservCommands.PWM, [parseInt(left), parseInt(right), parseInt(ms)], function() { this.fifo.orderFinished(); }.bind(this));
    }

    goxy(x, y, direction) {
        var directionInt;
        if(direction == "forward") directionInt = 1;
        else if(direction == "backward") directionInt = -1;
        else directionInt = 0;
        var posToArduino = this.robot.posIaToArduino({
            x: parseInt(x),
            y: parseInt(y)
        });
        this.ordersSerial.sendOrder(this.asservCommands.GOTO, [posToArduino.x, posToArduino.y, parseInt(directionInt)], function() { this.fifo.orderFinished(); }.bind(this));
    }

    goa(a, callback){
        var posToArduino = this.robot.posIaToArduino({
            a: this.myWriteFloat(a)
        });
        this.ordersSerial.sendOrder(this.asservCommands.ROT, [posToArduino.a], function() { this.fifo.orderFinished(); }.bind(this));
    }

    setPid(p, i, d){
        this.ordersSerial.sendOrder(this.asservCommands.PIDALL, [this.myWriteFloat(p), this.myWriteFloat(i), this.myWriteFloat(d)], function() { this.fifo.orderFinished(); }.bind(this));
    }

    doStartSequence(params){
        this.goxy(params.x, params.y, params.direction);
        this.goa(params.a, () => {});
    }

    setEmergencyStop (activate) {
        var value = activate?1:0;
        this.ordersSerial.sendOrder (
            this.asservCommands.SETEMERGENCYSTOP,
            [value],
            () => {this.fifo.orderFinished();}
        );
        super.setEmergencyStop(activate);
    }

    pause () {
        this.ordersSerial.sendOrder(this.asservCommands.PAUSE, [], function() { this.fifo.orderFinished(); }.bind(this));
    }

    resume () {
        this.ordersSerial.sendOrder(this.asservCommands.RESUME, [], function() { this.fifo.orderFinished(); }.bind(this));
    }

    /********************************************************************\
     *
     *  HELPERS FUNCTIONS
     *
     /********************************************************************/

    // For float
    myWriteFloat(f){ return Math.round(f*this.asservCommands.FLOAT_PRECISION); }
    myParseFloat(f){ return parseInt(f)/this.asservCommands.FLOAT_PRECISION; }

}

// Exports an object to be sure to have a single instance in the system
module.exports = function(robot, serialPort) {
    return new AsservReal(robot, serialPort);
};
