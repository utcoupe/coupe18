//
// Created by tfuhrman on 23/04/17.
//

#include "servo_motors.h"
#include "parameters.h"
#include "sender.h"
#include <Servo.h>
#include <Arduino.h>
#include "color_sensor_tcs3200.h"
#include <Timer.h>

//todo create a real servo object ??

void servoApplyCommand(uint8_t servo_id, uint8_t value, uint16_t order_id);

//todo min & max values for all servo

Servo pr_module_arm;
Servo pr_module_drop_r;
Servo pr_module_drop_l;
Servo pr_module_rotate;

// Parameters are : INIT, OPEN, CLOSE, ACTION_TIME(ms)
// INIT, OPEN and CLOSE are PWM values for position of the servo motor (0 - 180 max)
uint16_t servoValues[5][4] = {
        {90, 0, 165, 1000},      //PR_MODULE_ARM
        {10, 90, 5, 1000},      //PR_MODULE_DROP_R
        {180, 90, 180, 1000},       //PR_MODULE_DROP_L
        {90, 180, 255, 200},       //PR_MODULE_ROTATE
        {0, 1, 0, 500}
};

//todo adjust timer time
Timer armTimer = Timer(servoValues[PR_MODULE_ARM][TIMER], &servoArmCallback);
Timer armRotateTimer = Timer(servoValues[PR_MODULE_ARM_ROTATE][TIMER], &servoArmRotateCallback);
Timer dropRTimer = Timer(servoValues[PR_MODULE_DROP_R][TIMER], &servoDropRCallback);
Timer dropLTimer = Timer(servoValues[PR_MODULE_DROP_L][TIMER], &servoDropLCallback);
Timer rotateTimer = Timer(servoValues[PR_MODULE_ROTATE][TIMER], &servoRotateCallback);

//todo dynamic structure with mapping servo_id - order_id ?
// 0 is the default value, means no order
uint16_t armLastId = 0;
uint16_t armRotateLastId = 0;
uint16_t dropRLastId = 0;
uint16_t dropLLastId = 0;
uint16_t servoRotateLastId = 0;
MODULE_COLOR servoRotateColor = WHATEVER;

void servoAttach() {
    pr_module_arm.attach(PR_MODULE_ARM_PIN);
    pr_module_drop_r.attach(PR_MODULE_DROP_R_PIN);
    pr_module_drop_l.attach(PR_MODULE_DROP_L_PIN);
    pr_module_rotate.attach(PR_MODULE_ROTATE_PIN);
    // Setup the motor on the arm
    pinMode(PR_MODULE_ARM_ROTATE_PIN, OUTPUT);
    // Apply default values
    pr_module_arm.write(servoValues[PR_MODULE_ARM][OPEN]); //put the open value to avoid conflict with the module grabber
    pr_module_drop_r.write(servoValues[PR_MODULE_DROP_R][INIT]);
    pr_module_drop_l.write(servoValues[PR_MODULE_DROP_L][INIT]);
    pr_module_rotate.write(servoValues[PR_MODULE_ROTATE][INIT]);
}

void servoAction(uint8_t servo_id, SERVO_POSITION position, uint16_t order_id) {
    //todo maximal servo id as define
    if ((servo_id < 5) && (position < NB_POS)) {
        servoApplyCommand(servo_id, servoValues[servo_id][position], order_id);
    } else {
        SerialSender::SerialSend(SERIAL_INFO, "Servo %d doesn't exist or position %d is unknown...", servo_id, position);
    }
}

//todo put the timer start in another function to avoid multiple call ba protocol.cpp
void servoApplyCommand(uint8_t servo_id, uint8_t value, uint16_t order_id) {
    if (value < MAX_UINT8_T_VALUE) {
        switch (servo_id) {
            case PR_MODULE_ARM:
                armTimer.Start();
                armLastId = order_id;
                pr_module_arm.write(value);
                break;
            case PR_MODULE_ARM_ROTATE: 
                armRotateTimer.Start();
                armRotateLastId = order_id;
                switch(value){
                    case 0 : //open
                        digitalWrite(PR_MODULE_ARM_ROTATE_PIN, LOW);
                        break;
                    case 1: //close, init
                        digitalWrite(PR_MODULE_ARM_ROTATE_PIN, HIGH);
                        break;
                    default:
                        break;
                }
                break;
            case PR_MODULE_DROP_R:
                dropRTimer.Start();
                dropRLastId = order_id;
                pr_module_drop_r.write(value);
                break;
            case PR_MODULE_DROP_L:
                dropLTimer.Start();
                dropLLastId = order_id;
                pr_module_drop_l.write(value);
                break;
            case PR_MODULE_ROTATE:
                servoRotateLastId = order_id;
                pr_module_rotate.write(value);
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
    //todo servo id as define
    if ((servo_id < MAX_SERVO) && (servo_position < NB_POS) && (servo_position < MAX_UINT8_T_VALUE)) {
        servoValues[servo_id][servo_position] = servo_value;
    } else {
        SerialSender::SerialSend(SERIAL_INFO, "Servo id (%d) or servo position (%d) is not correct", servo_id, servo_position);
    }
}

void servoRotate(MODULE_COLOR color, uint16_t order_id) {
    //if color is whatever, no need to rotate
    if (color > WHATEVER && color <= YELLOW) {
        // Activate rotation
        servoAction(PR_MODULE_ROTATE, OPEN, order_id);
        servoRotateColor = color;
        rotateTimer.Start();
    }
}

void servoRotateCallback() {
    if (servoRotateColor != WHATEVER) {
        if (computeColor() == servoRotateColor) {
            // Send order id to ack that arduino has process the order
            SerialSender::SerialSend(SERIAL_INFO, "%d;", servoRotateLastId);
            // Stop the rotate servo motor (id 0 because motor is stopped)
            servoAction(PR_MODULE_ROTATE, INIT, 0);
            // Put the global variable to default value
            servoRotateColor = WHATEVER;
            servoRotateLastId = 0;
            rotateTimer.Stop();
        }
    }
}

void servoArmCallback() {
    if (armLastId != 0) {
        // Send order id to ack that arduino has process the order
        SerialSender::SerialSend(SERIAL_INFO, "%d;", armLastId);
        armLastId = 0;
        armTimer.Stop();
    }
}

void servoArmRotateCallback() {
    if (armRotateLastId != 0) {
        // Send order id to ack that arduino has process the order
        SerialSender::SerialSend(SERIAL_INFO, "%d;", armRotateLastId);
        armRotateLastId = 0;
        armRotateTimer.Stop();
    }
}

void servoDropRCallback() {
    if (dropRLastId != 0) {
        // Send order id to ack that arduino has process the order
        SerialSender::SerialSend(SERIAL_INFO, "%d;", dropRLastId);
        dropRLastId = 0;
        dropRTimer.Stop();
    }
}

void servoDropLCallback() {
    if (dropLLastId != 0) {
        // Send order id to ack that arduino has process the order
        SerialSender::SerialSend(SERIAL_INFO, "%d;", dropLLastId);
        dropLLastId = 0;
        dropLTimer.Stop();
    }
}

void servoTimerUpdate() {
    armTimer.Update();
    dropRTimer.Update();
    dropLTimer.Update();
    rotateTimer.Update();
    armRotateTimer.Update();

}


/*pr_module_arm :
 * initial value : 90
 * open value : 0
 * close value : 90
 * push module value : 150
 */
