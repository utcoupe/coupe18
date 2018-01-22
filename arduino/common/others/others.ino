//Sensors includes
#include <Wire.h>
#include "VL53L0X.h"

//Actuators includes
#include "Servo.h"

//ROS includes
#include <ros.h>
#include <drivers_ard_others/BeltRange.h>
#include <drivers_ard_others/Move.h>
ros::NodeHandle nh;


// ---- SENSORS DEPARTMENT ----

#define NUM_BELT_SENSORS 2
const int pins_belt_sensors_shut[NUM_BELT_SENSORS]     = {2, 3};
const uint8_t belt_sensors_addresses[NUM_BELT_SENSORS] = {22, 25};
const String belt_sensors_names[NUM_BELT_SENSORS]      = {"sensor1", "sensor2"};
VL53L0X belt_sensors[NUM_BELT_SENSORS];

drivers_ard_others::BeltRange belt_range_msg;
ros::Publisher belt_ranges_pub("/drivers/ard_others/belt_ranges", &belt_range_msg);

void init_belt_sensors() {
    for(int i = 0; i < NUM_BELT_SENSORS; i++) {
        pinMode(pins_belt_sensors_shut[i], OUTPUT);
        digitalWrite(pins_belt_sensors_shut[i], LOW);
    }

    for(int i = 0; i < NUM_BELT_SENSORS; i++) {
        pinMode(pins_belt_sensors_shut[i], INPUT);
        delay(50);
        belt_sensors[i].init(true);
        delay(100);
        belt_sensors[i].setAddress(belt_sensors_addresses[i]);
    }

    for(int i = 0; i < NUM_BELT_SENSORS; i++)
        belt_sensors[i].setTimeout(500);
}

void loop_belt_sensors() {
    for(int i = 0; i < NUM_BELT_SENSORS; i++) {
        belt_range_msg.sensor_id = belt_sensors_names[i].c_str();
        belt_range_msg.range = belt_sensors[i].readRangeSingleMillimeters() / 1000.0; //in meters
        if (belt_range_msg.range > 65534)
            belt_range_msg.range = -1;
        belt_ranges_pub.publish(&belt_range_msg);
    }
}

void init_sensors() {
    init_belt_sensors();
}

void loop_sensors() {
    loop_belt_sensors();
}


// ---- ACTUATORS DEPARTMENT ----

#define NUM_DIGITAL_ACTUATORS 2
const int pins_digital_actuators[NUM_DIGITAL_ACTUATORS]     = {13};
bool digital_actuators_states[NUM_DIGITAL_ACTUATORS]        = {true};
// Names : main_led, power_relay

#define NUM_PWM_ACTUATORS 2
const int pins_pwm_actuators_pwm[NUM_PWM_ACTUATORS]         = {6, 7};
uint8_t pwm_actuators_values[NUM_PWM_ACTUATORS]             = {0, 255};
// Names : motor_canon1, motor_canon2

#define NUM_SERVO_ACTUATORS 1
const int pins_servo_actuators_pwm[NUM_SERVO_ACTUATORS]     = {9};
int servo_actuators_angles[NUM_SERVO_ACTUATORS]             = {90};
Servo servo_actuators_objects[NUM_SERVO_ACTUATORS];
// Names : servo_main_door

// Digital actuators
void init_digital_actuators() {
    for(int i = 0; i < NUM_DIGITAL_ACTUATORS; i++)
        pinMode(pins_digital_actuators[i], OUTPUT);
}

void loop_digital_actuators() {
    for(int i = 0; i < NUM_DIGITAL_ACTUATORS; i++) {
        nh.loginfo(digital_actuators_states[i]);
        digitalWrite(pins_digital_actuators[i], digital_actuators_states[i]);
    }
}

// PWM actuators
void init_pwm_actuators() {
    for(int i = 0; i < NUM_PWM_ACTUATORS; i++)
        pinMode(pins_digital_actuators[i], OUTPUT);
}

void loop_pwm_actuators() {
    for(int i = 0; i < NUM_PWM_ACTUATORS; i++)
        analogWrite(pins_digital_actuators[i], pwm_actuators_values[i]);
}

// Servo actuators
void init_servo_actuators() {
    for(int i = 0; i < NUM_SERVO_ACTUATORS; i++) {
        pinMode(pins_digital_actuators[i], OUTPUT);
        servo_actuators_objects[i].attach(pins_servo_actuators_pwm[i]);
    }
}

void loop_servo_actuators() {
    for(int i = 0; i < NUM_SERVO_ACTUATORS; i++) {
        servo_actuators_objects[i].write(servo_actuators_angles[i]);
    }
}

void init_actuators() {
    init_digital_actuators();
    init_pwm_actuators();
    init_servo_actuators();
}

// Actuators callbacks
void on_move(const drivers_ard_others::Move& msg){
    switch(msg.type) {
        case msg.TYPE_DIGITAL:
            if(msg.id >= 0 && msg.id <= NUM_DIGITAL_ACTUATORS) {
                if(msg.dest_value == 0 || msg.dest_value == 1) {
                    nh.loginfo("digitaaaal");
                    digital_actuators_states[msg.id] = bool(msg.dest_value);
                }
            }
            break;
        case msg.TYPE_PWM:
            if(msg.id >= 0 && msg.id <= NUM_PWM_ACTUATORS)
                pwm_actuators_values[msg.id] = msg.dest_value;
            break;
        case msg.TYPE_SERVO:
            if(msg.id >= 0 && msg.id <= NUM_SERVO_ACTUATORS)
                servo_actuators_angles[msg.id] = msg.dest_value;
            break;
    }
}
ros::Subscriber<drivers_ard_others::Move> sub_move("/drivers/ard_others/move", &on_move);

void loop_actuators() {
    loop_digital_actuators();
    loop_pwm_actuators();
    loop_servo_actuators();
}

// ---- MAIN FUCNTIONS ----

void setup() {
    // ROS init
    nh.initNode();
    nh.advertise(belt_ranges_pub);
    nh.subscribe(sub_move);    

    // Libs init
    Wire.begin();

    // Components init
    init_sensors();
    init_actuators();

    nh.loginfo("Node '/arduinos/others' initialized correctly.");
}

void loop() {
    // Components loop
    loop_sensors();
    loop_actuators();

    // ROS loop
    nh.spinOnce();
    delay(50); //random delay for now
}
