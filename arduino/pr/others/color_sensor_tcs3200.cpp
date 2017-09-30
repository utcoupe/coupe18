//
// Created by tfuhrman on 27/04/17.
// Based on Arduino Color Sensing Tutorial
// by Dejan Nedelkovski, www.HowToMechatronics.com
//

#include "color_sensor_tcs3200.h"
#include <Arduino.h>
#include "sender.h"

#define S0          7
#define S1          8
#define S2          12
#define S3          13
#define LED         2
#define sensorOut   10

#define COLOR_ACCUMULATE_NB     1
#define COLOR_SENSOR_TIMEOUT    100000//in µs

#define WHITE_COLOR_THRESHOLD   600
#define YELLOW_COLOR_THRESHOLD  270

#define YELLOW_MIN_THRESHOLD    40
#define YELLOW_MAX_THRESHOLD    70
#define BLUE_MIN_THRESHOLD    180
#define BLUE_MAX_THRESHOLD    200
#define WHITE_MIN_THRESHOLD    0
#define WHITE_MAX_THRESHOLD    20

//red, green, blue
uint8_t rgbValues[3];
//use tlc values instead of rgb, because more easy to get colors
//uint16_t tslValues[3];

//used to map rawFrequency read from sensor to a RGB value on 8 bytes
//those data have to be calibrated to be optimal
//the index is rgbValuesName
uint8_t rgbMinMaxFrequency[3][2] = {
        {25, 54},
        {15, 100},
        {10, 90}
};

void setupColorSensor() {
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(sensorOut, INPUT);

    // Setting frequency-scaling to 20%
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);

    //todo power on the led when needed and power it off
    digitalWrite(LED, HIGH);

    //todo power on color sensor when needed otherwise power it off
}

void colorSensorValuesCapture() {
    //todo IR filter ?
    uint8_t rawFrequency = 0;
    for (uint8_t color_id = 0; color_id < 3; color_id++) {
        colorSensorFilterApply((rgbValuesName)color_id);
        rawFrequency = pulseIn(sensorOut, LOW);
        if (rawFrequency == 0) {
            SerialSender::SerialSend(SERIAL_ERROR, "Color sensor timed out...");
        } else {
            rgbValues[color_id] = constrain(map(rawFrequency, rgbMinMaxFrequency[color_id][0], rgbMinMaxFrequency[color_id][1], 255, 0), 0, 255);
        }
    }
}

MODULE_COLOR computeColor() {
    MODULE_COLOR returnValue = WHATEVER;
    uint16_t rgbColorAccumulator[3] = {0, 0, 0};
    uint8_t rgbMeanValues[3] = {0, 0, 0};
    uint16_t tslValues[3] = {0, 0, 0};
    // First accumulate color sensor values to be more accurate
    for (uint8_t accumulator_nb = 0; accumulator_nb < COLOR_ACCUMULATE_NB; accumulator_nb++) {
        colorSensorValuesCapture();
        rgbColorAccumulator[RGB_RED] += rgbValues[RGB_RED];
        rgbColorAccumulator[RGB_GREEN] += rgbValues[RGB_GREEN];
        rgbColorAccumulator[RGB_BLUE] += rgbValues[RGB_BLUE];
    }
    // Get the mean of the accumulated rgb data
    rgbMeanValues[RGB_RED] = rgbColorAccumulator[RGB_RED] / COLOR_ACCUMULATE_NB;
    rgbMeanValues[RGB_GREEN] = rgbColorAccumulator[RGB_GREEN] / COLOR_ACCUMULATE_NB;
    rgbMeanValues[RGB_BLUE] = rgbColorAccumulator[RGB_BLUE] / COLOR_ACCUMULATE_NB;
    SerialSender::SerialSend(SERIAL_DEBUG, "RED : %d, GREEN : %d, BLUE : %d", rgbMeanValues[RGB_RED], rgbMeanValues[RGB_GREEN], rgbMeanValues[RGB_BLUE]);
    // Compute the corresponding tsl colors
    computeTslColors(rgbMeanValues, tslValues);
    SerialSender::SerialSend(SERIAL_DEBUG, "HUE : %d, SATURATION : %d, LIGHTNESS : %d", tslValues[TSL_HUE], tslValues[TSL_SATURATION], tslValues[TSL_LIGHTNESS]);
//    // Compute the non rgb colors
//    uint16_t yellowColor = rgbMeanValues[RGB_RED] + rgbMeanValues[RGB_GREEN];
//    uint16_t colorSum = yellowColor + rgbMeanValues[RGB_BLUE];
//    String color;
//    //todo return the color corresponding to the protocol
//    if (colorSum > WHITE_COLOR_THRESHOLD) {
//        color = String("white");
//    } else {
//        //todo find a way to avoid blue -> white turning in yellow...
//        if (yellowColor > YELLOW_COLOR_THRESHOLD) {
//            color = String("yellow");
//            returnValue = YELLOW;
//        } else if ((yellowColor >> 2) < rgbMeanValues[RGB_BLUE]) {
//            color = String("blue");
//            returnValue = BLUE;
//        } else {
//            color = String("undefined");
//        }
//    }
    // Compute the color value
    String color;
    if ((tslValues[TSL_HUE] > YELLOW_MIN_THRESHOLD) && (tslValues[TSL_HUE] < YELLOW_MAX_THRESHOLD)) {
        color = String("yellow");
        returnValue = YELLOW;
    } else if ((tslValues[TSL_HUE] > BLUE_MIN_THRESHOLD) && (tslValues[TSL_HUE] < BLUE_MAX_THRESHOLD)) {
        returnValue = BLUE;
        color = String("blue");
    } else if ((tslValues[TSL_HUE] > WHITE_MIN_THRESHOLD) && (tslValues[TSL_HUE] < WHITE_MAX_THRESHOLD)) {
        color = String("white");
    } else {
        color = String("undefined");
    }
    SerialSender::SerialSend(SERIAL_INFO, color);
    return returnValue;
}

void colorSensorFilterApply(rgbValuesName color) {
    switch (color) {
        case RGB_RED:
            digitalWrite(S2,LOW);
            digitalWrite(S3,LOW);
            break;
        case RGB_GREEN:
            digitalWrite(S2,HIGH);
            digitalWrite(S3,HIGH);
            break;
        case RGB_BLUE:
            digitalWrite(S2,LOW);
            digitalWrite(S3,HIGH);
            break;
        default:
            break;
    }
}

void computeTslColors(uint8_t rgbValues[3], uint16_t tslColors[3]) {
    uint16_t maxColorValue = (uint16_t)rgbValues[RGB_RED];
    uint16_t minColorValue  = (uint16_t)rgbValues[RGB_RED];
    uint8_t maxColorValueIndex = 0;
    // Compute min and max color
    for (uint8_t index = 1; index < 3; index++) {
        if ((uint16_t)rgbValues[index] > maxColorValue) {
            maxColorValue = (uint16_t)rgbValues[index];
            maxColorValueIndex = index;
        }
        if ((uint16_t)rgbValues[index] < minColorValue) {
            minColorValue = (uint16_t)rgbValues[index];
        }
    }
    // Use float values, not best idea but standard TSL calculation can't be done in integers...
    uint16_t delta = maxColorValue - minColorValue;
    if (delta != 0) {
        float deltaf = (float)delta;
        float hue;
        if ((rgbValuesName)maxColorValueIndex == RGB_RED)
        {
            hue = (float)(rgbValues[RGB_GREEN] - rgbValues[RGB_BLUE]) / delta;
        }
        else
        {
            if ((rgbValuesName)maxColorValueIndex == RGB_GREEN)
            {
                hue = 2.0 + (float)(rgbValues[RGB_BLUE] - rgbValues[RGB_RED]) / delta;
            }
            else
            {
                hue = 4.0 + (float)(rgbValues[RGB_RED] - rgbValues[RGB_GREEN]) / delta;
            }
        }
        hue *= 60.0;
        if (hue < 0) hue += 360;
        tslColors[TSL_HUE] = (uint16_t)hue;
//        switch ((rgbValuesName)maxColorValueIndex) {
//            case RGB_RED:
//            {
//                tslColors[TSL_HUE] = (uint16_t)((uint16_t)60 * (uint16_t)((((uint16_t)rgbValues[RGB_GREEN] - (uint16_t)rgbValues[RGB_BLUE]) / (maxColorValue - minColorValue)) % 6));
//                break;
//            }
//            case RGB_GREEN:
//            {
//                tslColors[TSL_HUE] = (uint16_t)((uint16_t)60 * (uint16_t)((((uint16_t)rgbValues[RGB_BLUE] - (uint16_t)rgbValues[RGB_RED]) / (maxColorValue - minColorValue)) + (uint16_t)2));
//                break;
//            }
//            case RGB_BLUE:
//            {
//                tslColors[TSL_HUE] = (uint16_t)((uint16_t)60 * ((((uint16_t)rgbValues[RGB_RED] - (uint16_t)rgbValues[RGB_GREEN]) / (maxColorValue - minColorValue)) + (uint16_t)4));
//                break;
//            }
//            default:
//                SerialSender::SerialSend(SERIAL_ERROR, "ComputeTslColor : Color index %d does not exists...", maxColorValueIndex);
//                break;
//        }
    }
    // Compute the tsl values
    tslColors[TSL_SATURATION] = (uint16_t)((uint16_t)100 * (uint16_t)((maxColorValue - minColorValue)) / (uint16_t)maxColorValue);
    tslColors[TSL_LIGHTNESS] = (uint16_t)((uint16_t)100 * maxColorValue) / (uint16_t)255;
//    uint16_t maxin = (uint16_t)(maxColorValue - minColorValue);
//    uint16_t mult = (uint16_t)100 * maxin;
//    uint16_t div = mult / maxColorValue;
//    SerialSender::SerialSend(SERIAL_INFO, "Max = %d, 100Max = %d, light = %d, LIGHT = %d", maxColorValue, maxin, mult, tslColors[TSL_SATURATION]);
}
