//
// Created by tfuhrman on 13/04/18.
//

#ifndef ARDUINO_CONFIG_H
#define ARDUINO_CONFIG_H

#define SENSOR_BELT_ENABLED
#define SENSOR_COLOR_ENABLED

//********************************************************************************************************************//
//
// Belt sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_BELT_ENABLED
//TODO all this stuff !
#endif

//********************************************************************************************************************//
//
// Color sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_COLOR_ENABLED

// Pins
#define S0              8
#define S1              9
#define S2              10
#define S3              11
#define LED             2
#define SENSOR_VALUE    12

// Component configuration
#define COLOR_ACCUMULATE_NB     1
#define COLOR_SENSOR_TIMEOUT    100000//in µs

#endif

#endif //ARDUINO_CONFIG_H
