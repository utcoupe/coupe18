//Adapted from Adafruit Motor Shield by Quentin C for UTCoupe
//Defined for brushless controler shield designed by UTCoupe
//08/04/2013

#ifndef BRUSHLESSMOTOR_H
#define BRUSHLESSMOTOR_H

#include "parameters.h"
#include "pins.h"

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

#define LEFT_READY_SHIFT 1
#define RIGHT_READY_SHIFT 2

#define LEFT_READY (1<<LEFT_READY_SHIFT)
#define RIGHT_READY (1<<RIGHT_READY_SHIFT)

void BrushlessMotorsInit();
int BrushlessMotorsReady();
void BrushlessMotorSetPwm(int motor_side, int pwm);

#endif
