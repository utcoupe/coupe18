#include <Wire.h>
#include <ros.h>
#include "VL53L0X.h"
#include <drivers_ard_others/BeltRange.h>
ros::NodeHandle nh;

const int num_belt_sensors                             = 2; // sensors count
const int belt_sensors_shut_pins[num_belt_sensors]     = {9, 10};
const uint8_t belt_sensors_addresses[num_belt_sensors] = {22, 25};
const String belt_sensors_names[num_belt_sensors]      = {"sensor1", "sensor2"};
VL53L0X belt_sensors[num_belt_sensors];

drivers_ard_others::BeltRange belt_range_msg;
ros::Publisher belt_ranges_pub("/drivers/ard_others/belt_ranges", &belt_range_msg);

void belt_init() {
    for(int i = 0; i < num_belt_sensors; i++) {
        pinMode(belt_sensors_shut_pins[i], OUTPUT);
        digitalWrite(belt_sensors_shut_pins[i], LOW);
    }

    for(int i = 0; i < num_belt_sensors; i++) {
        pinMode(belt_sensors_shut_pins[i], INPUT);
        delay(50);
        belt_sensors[i].init(true);
        delay(100);
        belt_sensors[i].setAddress(belt_sensors_addresses[i]);
    }

    for(int i = 0; i < num_belt_sensors; i++)
        belt_sensors[i].setTimeout(500);
}

void process_belt() {
    nh.logwarn("publishing...");

    for(int i = 0; i < num_belt_sensors; i++) {
        belt_range_msg.sensor_id = belt_sensors_names[i].c_str();
        belt_range_msg.range = belt_sensors[i].readRangeSingleMillimeters() / 1000.0; //in meters
        if (belt_range_msg.range > 65534)
            belt_range_msg.range = -1;
        belt_ranges_pub.publish(&belt_range_msg);
    }
}

void setup() {
    nh.initNode();
    nh.advertise(belt_ranges_pub);
    nh.loginfo("Started ard_others");

    Wire.begin();

    belt_init();

    nh.logwarn("Init done.");
}

void loop() {
    process_belt();
    nh.spinOnce();
    delay(50); //random delay for now
}
