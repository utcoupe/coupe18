//
// Created by tfuhrman & Elwan Héry on 29/04/18.
//

#ifndef ARDUINO_ACTUATORS_H
#define ARDUINO_ACTUATORS_H

#include <stdint.h>

//********************************************************************************************************************//
//
// Regulated actuators
//
//********************************************************************************************************************//

#ifdef REGULATED_ACTUATORS_ENABLED

void init_regulated_actuators();
void loop_regulated_actuators();
void activate_regulated_actuators(float reference_value);
void deactivate_regulated_actuators();

#endif

#endif //ARDUINO_ACTUATORS_H