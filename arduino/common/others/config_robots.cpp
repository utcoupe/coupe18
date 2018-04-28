//
// Created by tfuhrman on 28/04/18.
//

#include "config_robots.h"
#include "stdint.h"

// TODO this way to do is kind of ugly, think how to not duplicate variable declaration
// TODO find a way to just declares pins as define and to automatically fill the variable structures

//********************************************************************************************************************//
//
// PR ROBOT
//
//********************************************************************************************************************//

#if defined(PR_ROBOT) && !defined(GR_ROBOT)

uint8_t pins_digital_actuators[NUM_DIGITAL_ACTUATORS]   = {12};
bool digital_actuators_states[NUM_DIGITAL_ACTUATORS]    = {true};
// Names : main_led

uint8_t pins_pwm_actuators_pwm[NUM_PWM_ACTUATORS]       = {8};
uint8_t pwm_actuators_states[NUM_PWM_ACTUATORS]         = {0};
// Names : canon

uint8_t pins_servo_actuators_pwm[NUM_SERVO_ACTUATORS]   = {9,  10, 11};
int16_t servo_actuators_states[NUM_SERVO_ACTUATORS]     = {10, 10, 10};
// Names : servo_front_lift, servo_back_lift, servo_lock

int16_t stepper_actuators_states[NUM_STEPPER_ACTUATORS] = {};
// Names :

#endif

//********************************************************************************************************************//
//
// GR ROBOT
//
//********************************************************************************************************************//

#if defined(GR_ROBOT) && !defined(PR_ROBOT)

uint8_t pins_digital_actuators[NUM_DIGITAL_ACTUATORS]   = {};
bool digital_actuators_states[NUM_DIGITAL_ACTUATORS]    = {};
// Names :

uint8_t pins_pwm_actuators_pwm[NUM_PWM_ACTUATORS]       = {};
uint8_t pwm_actuators_states[NUM_PWM_ACTUATORS]         = {};
// Names :

uint8_t pins_servo_actuators_pwm[NUM_SERVO_ACTUATORS]   = {37};
int16_t servo_actuators_states[NUM_SERVO_ACTUATORS]     = {10};
// Names :

int16_t stepper_actuators_states[NUM_STEPPER_ACTUATORS] = {0};
// Names :

#endif