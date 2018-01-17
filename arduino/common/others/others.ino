#include <Wire.h>
#include <ros.h>
#include "VL53L0X.h"
#include <drivers_ard_others/BeltRange.h>
ros::NodeHandle nh;


// BELT SENSORS
//Adafruit_VL53L0X lox = Adafruit_VL53L0X(); //tmp
VL53L0X sensor;

const int NUM_BELT_SENSORS = 6;
const String belt_sensors_names[] = {"sensor_top_left", "sensor_top", "sensor_top_right",
                                     "sensor_bot_left", "sensor_bot", "sensor_bot_right"};
const int belt_sensors_addresses[] = {0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
VL53L0X belt_sensors[NUM_BELT_SENSORS];

drivers_ard_others::BeltRange belt_range_msg;
ros::Publisher belt_ranges_pub("/drivers/ard_others/belt_ranges", &belt_range_msg);

void process_belt() {
    //VL53L0X_RangingMeasurementData_t measure;
    //lox.rangingTest(&measure, false); //false or true for debug printout

//    for(int i = 0; i < NUM_BELT_SENSORS; i++) {
//        belt_range_msg.sensor_id = belt_sensors_names[i].c_str();
//        if(measure.RangeStatus != 4) belt_range_msg.range = measure.RangeMilliMeter;
//        else belt_range_msg.range = -1;
//
//        belt_ranges_pub.publish(&belt_range_msg);
//    }

    //TODO use a loop and multiple sensors
    belt_range_msg.sensor_id = "sensor1";
    belt_range_msg.range = sensor.readRangeSingleMillimeters(); //in millimeters
    if (belt_range_msg.range > 65534) {
      belt_range_msg.range = -1;
    }
    belt_ranges_pub.publish(&belt_range_msg);
}

void setup() {
    nh.initNode();
    nh.advertise(belt_ranges_pub);
    nh.loginfo("Started ard_others");

    Wire.begin();
    sensor.init();
	  sensor.setTimeout(500);

    // Configure the sensor in single shot long range
    //sensor.setSignalRateLimit(0.1);
    //sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    //sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(20000);

//    lox.setAddress(0x34);
//    if (!lox.begin()) {
//        nh.logerror("[OTHERS] Could not init belt sensor.");
//    }
}

void loop() {
    process_belt();
    nh.spinOnce();
    delay(50); //random delay for now
}
