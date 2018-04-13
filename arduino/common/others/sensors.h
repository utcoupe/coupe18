//
// Created by tfuhrman on 13/04/18.
//

#ifndef ARDUINO_SENSOR_H
#define ARDUINO_SENSOR_H

#include "config.h"
#include <Arduino.h>

//********************************************************************************************************************//
//
// Color sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_COLOR_ENABLED

// Public interface
enum rgb_name_enum {
    RGB_RED = 0,
    RGB_GREEN,
    RGB_BLUE
};

enum tsl_name_enum {
    TSL_HUE = 0,
    TSL_SATURATION,
    TSL_LIGHTNESS
};

void color_sensor_init();
void color_sensor_loop();

// Private stuff
void color_sensor_capture_values();
void color_sensor_apply_filter(rgb_name_enum color);
void color_sensor_rgb_to_tsl(uint8_t rgbValues[3], uint16_t tslColors[3]);
void color_sensor_publish_values(uint16_t tslColors[3]);

#endif

#endif //ARDUINO_SENSOR_H