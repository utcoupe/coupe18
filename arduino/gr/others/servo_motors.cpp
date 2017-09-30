//
// Created by tfuhrman on 23/04/17.
//

#include "servo_motors.h"
#include "parameters.h"
#include "sender.h"
#include <Servo.h>
#include <Arduino.h>
#include <Timer.h>

//todo create a real servo object ??

void servoApplyCommand(uint8_t servo_id, uint8_t value, uint16_t order_id);

//todo min & max values for all servo

Servo gr_rocket;
Servo gr_loader;

// Parameters are : INIT, OPEN, CLOSE, ACTION_TIME(ms)
// INIT, OPEN and CLOSE are PWM values for position of the servo motor (0 - 180 max)
uint8_t servoValues[4][4] = {
        {0, 180, 0, 100},       //GR_SWEEPER
        {0, 33, 0, 100},        //GR_CANON
        {49, 70, 49, 100},       //GR_ROCKET
        {100, 180, 100, 100},    //GR_LOADER
};

//todo adjust timer time
Timer rocketTimer = Timer(servoValues[GR_ROCKET][TIMER], &servoRocketCallback);
Timer loaderTimer = Timer(servoValues[GR_LOADER][TIMER], &servoLoaderCallback);

//todo dynamic structure with mapping servo_id - order_id ?
// 0 is the default value, means no order
uint16_t rocketLastId = 0;
uint16_t loaderLastId = 0;

void servoAttach() {
    gr_rocket.attach(GR_ROCKET_PIN);
    gr_loader.attach(GR_LOADER_PIN);
    // Use regular pins for sweeper and canon
    pinMode(GR_SWEEPER_PIN, OUTPUT);
    pinMode(GR_CANON_PIN, OUTPUT);
    // Apply default values
    gr_rocket.write(servoValues[GR_ROCKET][INIT]);
    gr_loader.write(servoValues[GR_LOADER][INIT]);
    analogWrite(GR_SWEEPER_PIN, 0);
    analogWrite(GR_CANON_PIN, 0);
}

void servoAction(uint8_t servo_id, SERVO_POSITION position, uint16_t order_id) {
    if ((servo_id < MAX_SERVO) && (position < NB_POS)) {
        servoApplyCommand(servo_id, servoValues[servo_id][position], order_id);
    } else {
        SerialSender::SerialSend(SERIAL_INFO, "Servo %d doesn't exist or position %d is unknown...", servo_id, position);
    }
}

//todo put the timer start in another function to avoid multiple call ba protocol.cpp
void servoApplyCommand(uint8_t servo_id, uint8_t value, uint16_t order_id) {
    if (value < MAX_UINT8_T_VALUE) {
        switch (servo_id) {
            case GR_SWEEPER:
                analogWrite(GR_SWEEPER_PIN, value);
                SerialSender::SerialSend(SERIAL_INFO, "%d;", order_id);
                break;
            case GR_CANON:
                analogWrite(GR_CANON_PIN, value);
                SerialSender::SerialSend(SERIAL_INFO, "%d;", order_id);
                break;
            case GR_ROCKET:
                rocketTimer.Start();
                rocketLastId = order_id;
                gr_rocket.write(value);
                break;
            case GR_LOADER:
                loaderTimer.Start();
                loaderLastId = order_id;
                gr_loader.write(value);
                break;
            default:
                SerialSender::SerialSend(SERIAL_INFO, "Servo %d doesn't exist...", servo_id);
                break;
        }
    } else {
        SerialSender::SerialSend(SERIAL_INFO, "Value %d for servo %d doesn't exist...", value, servo_id);
    }
}

void servoChangeParameter(const uint8_t servo_id, const SERVO_POSITION servo_position, const uint8_t servo_value) {
    SerialSender::SerialSend(SERIAL_INFO, "servo=%d, pos=%d, value=%d", servo_id, servo_position, servo_value);
    if ((servo_id < MAX_SERVO) && (servo_position < NB_POS) && (servo_position < MAX_UINT8_T_VALUE)) {
        servoValues[servo_id][servo_position] = servo_value;
    } else {
        SerialSender::SerialSend(SERIAL_INFO, "Servo id (%d) or servo position (%d) is not correct", servo_id, servo_position);
    }
}

void servoRocketCallback() {
    if (rocketLastId != 0) {
        // Send order id to ack that arduino has process the order
        SerialSender::SerialSend(SERIAL_INFO, "%d;", rocketLastId);
        rocketLastId = 0;
        rocketTimer.Stop();
    }
}

void servoLoaderCallback() {
    if (loaderLastId != 0) {
        // Send order id to ack that arduino has process the order
        SerialSender::SerialSend(SERIAL_INFO, "%d;", loaderLastId);
        loaderLastId = 0;
        loaderTimer.Stop();
    }
}

void servoTimerUpdate() {
    rocketTimer.Update();
    loaderTimer.Update();
}
