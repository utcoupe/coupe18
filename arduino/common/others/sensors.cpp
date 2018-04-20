//
// Created by tfuhrman on 13/04/18.
//

#include "sensors.h"
#include <drivers_ard_others/Color.h>

//********************************************************************************************************************//
//
// Color sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_COLOR_ENABLED

ros::NodeHandle* node_handle = NULL;

drivers_ard_others::Color color_msg;
ros::Publisher color_pub("/drivers/ard_others/color", &color_msg);

uint8_t color_sensor_rgb_values[3];

//used to map rawFrequency read from sensor to a RGB value on 8 bytes
//those data have to be calibrated to be optimal
//the index is rgb_name_enum
uint8_t rgbMinMaxFrequency[3][2] = {
        {25, 54},
        {15, 100},
        {10, 90}
};

void color_sensor_init(ros::NodeHandle* nh) {
    node_handle = nh;
    if (node_handle) {
        node_handle->advertise(color_pub);
    }
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(SENSOR_VALUE, INPUT);
    // Setting frequency-scaling to 20%
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);
    // TODO LED never powered off
    digitalWrite(LED, HIGH);
}

void color_sensor_loop() {
    uint16_t rgbColorAccumulator[3] = {0, 0, 0};
    uint8_t rgbMeanValues[3] = {0, 0, 0};
    uint16_t tslValues[3] = {0, 0, 0};
    // First accumulate color sensor values to be more accurate
    for (uint8_t accumulator_nb = 0; accumulator_nb < COLOR_ACCUMULATE_NB; accumulator_nb++) {
        color_sensor_values_capture();
        rgbColorAccumulator[RGB_RED] += color_sensor_rgb_values[RGB_RED];
        rgbColorAccumulator[RGB_GREEN] += color_sensor_rgb_values[RGB_GREEN];
        rgbColorAccumulator[RGB_BLUE] += color_sensor_rgb_values[RGB_BLUE];
    }
    // Get the mean of the accumulated rgb data
    rgbMeanValues[RGB_RED] = rgbColorAccumulator[RGB_RED] / COLOR_ACCUMULATE_NB;
    rgbMeanValues[RGB_GREEN] = rgbColorAccumulator[RGB_GREEN] / COLOR_ACCUMULATE_NB;
    rgbMeanValues[RGB_BLUE] = rgbColorAccumulator[RGB_BLUE] / COLOR_ACCUMULATE_NB;
    // Compute the corresponding tsl colors
    color_sensor_rgb_to_tsl(rgbMeanValues, tslValues);
    // Finally send the data
    color_sensor_values_publish(tslValues);
}

void color_sensor_values_capture() {
    uint8_t rawFrequency = 0;
    for (uint8_t color_id = 0; color_id < 3; color_id++) {
        color_sensor_filter_apply((rgb_name_enum)color_id);
        rawFrequency = pulseIn(SENSOR_VALUE, LOW);
        if (rawFrequency > 0) {
            color_sensor_rgb_values[color_id] = constrain(map(rawFrequency, rgbMinMaxFrequency[color_id][0], rgbMinMaxFrequency[color_id][1], 255, 0), 0, 255);
        } else {
            node_handle->logwarn("Color sensor timed out, no values...");
        }
    }
}

void color_sensor_filter_apply(rgb_name_enum color) {
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

void color_sensor_rgb_to_tsl(uint8_t rgbValues[3], uint16_t tslColors[3]) {
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
        if ((rgb_name_enum)maxColorValueIndex == RGB_RED)
        {
            hue = (float)(rgbValues[RGB_GREEN] - rgbValues[RGB_BLUE]) / delta;
        }
        else
        {
            if ((rgb_name_enum)maxColorValueIndex == RGB_GREEN)
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
    }
    // Compute the tsl values
    tslColors[TSL_SATURATION] = (uint16_t)((uint16_t)100 * (uint16_t)((maxColorValue - minColorValue)) / (uint16_t)maxColorValue);
    tslColors[TSL_LIGHTNESS] = (uint16_t)((uint16_t)100 * maxColorValue) / (uint16_t)255;
}

void color_sensor_values_publish(uint16_t tslColors[3]) {
    color_msg.hue = tslColors[TSL_HUE];
    color_msg.saturation = tslColors[TSL_SATURATION];
    color_msg.lightness = tslColors[TSL_LIGHTNESS];
    if (node_handle) {
        color_pub.publish(&color_msg);
    }
}

#endif
