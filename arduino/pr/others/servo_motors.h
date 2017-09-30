//
// Created by tfuhrman on 23/04/17.
//

#ifndef ARDUINO_SERVO_MOTORS_H
#define ARDUINO_SERVO_MOTORS_H

#include "protocol.h"
#include <stdint.h>

void servoAttach();
void servoAction(uint8_t servo_id, SERVO_POSITION position, uint16_t order_id);
void servoChangeParameter(const uint8_t servo_id, const SERVO_POSITION servo_position, const uint8_t servo_value);
//color is the colo to reach
void servoRotate(MODULE_COLOR color, uint16_t order_id);
void servoTimerUpdate();
void servoRotateCallback();
//todo unique method with mapped structure ?
void servoArmCallback();
void servoDropRCallback();
void servoDropLCallback();
void servoArmRotateCallback();

#endif //ARDUINO_SERVO_MOTORS_H
