//
// Created by tfuhrman & Elwan Héry on 29/04/18.
//

#include "actuators.h"
#include "config_actuators.h"
#include <Arduino.h>

//********************************************************************************************************************//
//
// Regulated actuators
//
//********************************************************************************************************************//

#ifdef REGULATED_ACTUATORS_ENABLED

void interrupt_regulated_actuators();

unsigned long ticks_counter = 0;
uint8_t regulation_activated = 0;
float regulation_reference_value = 0;

void init_regulated_actuators() {
    pinMode(REGULATED_ACTUATORS_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(REGULATED_ACTUATORS_INTERRUPT_PIN), interrupt_regulated_actuators, RISING);
    pinMode(REGULATED_ACTUATORS_PIN, OUTPUT);
}

void loop_regulated_actuators() {
    static unsigned long last_control_time = 0;
    unsigned long current_control_time = millis();
    unsigned long delta_time = current_control_time - last_control_time;
    last_control_time = current_control_time;
    if (regulation_activated > 0) {
        if (delta_time >= REGULATED_ACTUATORS_CONTROL_LOOP_MS) {
            static unsigned long last_ticks = 0;
            static float sum_speed_error = 0.0;
            static float last_speed_error = 0.0;
            float wheel_speed = ((ticks_counter - last_ticks) / float(REGULATED_ACTUATORS_TICS_PER_REVOLUTION)) / (delta_time / 1000.0);
            last_ticks = ticks_counter;
            float speed_error = regulation_reference_value - wheel_speed;
            sum_speed_error += speed_error;
            int pwm_to_apply = REGULATED_ACTUATORS_PID_P * speed_error +
                               REGULATED_ACTUATORS_PID_I * sum_speed_error +
                               REGULATED_ACTUATORS_PID_D * (speed_error - last_speed_error);
            last_speed_error = speed_error;
            if (pwm_to_apply > REGULATED_ACTUATORS_PWM_MAX) {
                pwm_to_apply = REGULATED_ACTUATORS_PWM_MAX;
            } else if (pwm_to_apply < REGULATED_ACTUATORS_PWM_MIN) {
                pwm_to_apply = 0;
            }
            analogWrite(REGULATED_ACTUATORS_PIN, pwm_to_apply);
        }
    } else {
        analogWrite(REGULATED_ACTUATORS_PIN, 0);
    }
}

void activate_regulated_actuators(float reference_value) {
    regulation_reference_value = reference_value;
    regulation_activated = 1;
}

void deactivate_regulated_actuators() {
    regulation_activated = 0;
    regulation_reference_value = 0;
}

void interrupt_regulated_actuators() {
    ticks_counter++;
}

#endif