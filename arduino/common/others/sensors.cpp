//
// Created by tfuhrman on 13/04/18.
//

#include "sensors.h"

//********************************************************************************************************************//
//
// Color sensor
//
//********************************************************************************************************************//

#ifdef SENSOR_COLOR_ENABLED

uint8_t color_sensor_rgb_values[3];

void color_sensor_init() {
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

}

void color_sensor_capture_values() {

}

void color_sensor_apply_filter(rgb_name_enum color) {
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

}

void color_sensor_publish_values(uint16_t tslColors[3]) {

}

#endif