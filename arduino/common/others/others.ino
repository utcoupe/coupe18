#include <ros.h>
#include "Adafruit_VL53L0X.h"
#include <drivers_ard_others/BeltRangeList.h>
#include <drivers_ard_others/BeltRange.h>
ros::NodeHandle nh;


// BELT SENSORS
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const String belt_sensors_names[] = {"sensor_top_left", "sensor_top", "sensor_top_right",
                                     "sensor_bot_left", "sensor_bot", "sensor_bot_right"};
const int NUM_BELT_SENSORS = 6;
drivers_ard_others::BeltRange belt_ranges[NUM_BELT_SENSORS];

drivers_ard_others::BeltRangeList belt_rangelist_msg;
ros::Publisher belt_ranges_pub("/drivers/ard_others/belt_ranges", &belt_rangelist_msg);



void publish_belt_ranges() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); //false or true for debug printout

    for(int i = 0; i < NUM_BELT_SENSORS; i++) {
        drivers_ard_others::BeltRange range;
        range.sensor_id = belt_sensors_names[i].c_str();

        if(measure.RangeStatus != 4) 
            range.range = measure.RangeMilliMeter;
        else 
            range.range = -1;

        belt_rangelist_msg.sensors[i] = range;
    }
    
    belt_ranges_pub.publish(&belt_rangelist_msg);
}

void setup() {
    nh.initNode();
    nh.advertise(belt_ranges_pub);

    lox.setAddress(0x34);
    if (!lox.begin()) {
        nh.logerror("[OTHERS] Could not init belt sensor.");
        while(1);
    }
}

void loop() {
    publish_belt_ranges();
    nh.spinOnce();
    delay(100);
}