//
// Created by tfuhrman on 27/04/18.
//

#ifndef ARDUINO_PINS_H
#define ARDUINO_PINS_H

// You have to select a robot and only one !!!
//#define PR_ROBOT
#define GR_ROBOT

/*
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!                                                                              !!!
 * !!! DON'T FORGET TO PUT THE PIN NUMBERS AND INIT VALUES IN config_robot.cpp FILE !!!
 * !!!                                                                              !!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

// If a stepper shield is connected, don't use the following pins : 3 4 7 8 11 12 13

//********************************************************************************************************************//
//
// PR ROBOT
//
//********************************************************************************************************************//

#if defined(PR_ROBOT) && !defined(GR_ROBOT)

// ---- DIGITAL ACTUATORS ----
#define NUM_DIGITAL_ACTUATORS 1

// ---- PWM ACTUATORS ----
#define NUM_PWM_ACTUATORS 1

// ---- SERVO ACTUATORS ----
#define NUM_SERVO_ACTUATORS 3

// ---- STEPPER ACTUATORS ----
#define NUM_STEPPER_ACTUATORS 0

// ---- BELT SENSOR ----
#define SENSOR_BELT_ENABLED
#define NUM_BELT_SENSORS 2

// ---- COLOR SENSOR ----
#define SENSOR_COLOR_ENABLED
#define SENSOR_COLOR_S0              7
#define SENSOR_COLOR_S1              6
#define SENSOR_COLOR_S2              5
#define SENSOR_COLOR_S3              4
#define SENSOR_COLOR_LED             2
#define SENSOR_COLOR_SENSOR_VALUE    3

#endif

//********************************************************************************************************************//
//
// GR ROBOT
//
//********************************************************************************************************************//

#if defined(GR_ROBOT) && !defined(PR_ROBOT)

// ---- DIGITAL ACTUATORS ----
#define NUM_DIGITAL_ACTUATORS 0

// ---- PWM ACTUATORS ----
#define NUM_PWM_ACTUATORS 0

// ---- SERVO ACTUATORS ----
#define NUM_SERVO_ACTUATORS 1

// ---- STEPPER ACTUATORS ----
#define NUM_STEPPER_ACTUATORS 1

// ---- BELT SENSOR ----
#define SENSOR_BELT_ENABLED
#define NUM_BELT_SENSORS 2

#endif

#endif //ARDUINO_PINS_H
