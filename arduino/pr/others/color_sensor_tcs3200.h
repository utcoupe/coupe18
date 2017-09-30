//
// Created by tfuhrman on 27/04/17.
//

#ifndef ARDUINO_COLOR_SENSOR_TCS3200_H
#define ARDUINO_COLOR_SENSOR_TCS3200_H

#include "protocol.h"

enum rgbValuesName {
    RGB_RED = 0,
    RGB_GREEN,
    RGB_BLUE
};

enum tslValuesName {
    TSL_HUE = 0,
    TSL_SATURATION,
    TSL_LIGHTNESS
};

void setupColorSensor();
void colorSensorValuesCapture();
MODULE_COLOR computeColor();
void colorSensorFilterApply(rgbValuesName color);
void computeTslColors(uint8_t rgbValues[3], uint16_t tslColors[3]);

#endif //ARDUINO_COLOR_SENSOR_TCS3200_H
