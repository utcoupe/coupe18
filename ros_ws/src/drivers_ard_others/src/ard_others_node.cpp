#define Address1  0x00
#define Address2  0x00

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h> 

#include "processing_belt_interpreter/RangeList.h"

#include <Arduino.h>

ros::NodeHandle  nh;

std_msgs::Float32 value;
std_msgs::String str_msg;
processing_belt_interpreter::RangeList data_List;

ros::Publisher chatter("chatter", &str_msg);

// List of sensor publishers
ros::Publisher pub_sensor1("sensors/belt", &data_List);


//Sensor addresses
int sensorAddress1 = Address1;
int sensorAddress2 = Address2;

//Sensor data variable
int sensorPin = 11;
float sensorValue = 0;  // variable to store the value coming from the sensor


char hello[15] = "Test message!";

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(11, INPUT);

  //Wire.begin();        // join i2c bus (address optional for master)

  nh.initNode();

  // Advertise publishers
  nh.advertise(chatter);
  nh.advertise(pub_sensor1);

}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );

  // Get sensors' data
  sensorValue = analogRead(sensorPin);

  sensorValue = (sensorValue * 5 ) / 1023;

  // Publish sensors' data
  data_List.sensors[1].sensor_id = "Capteur 1";
  data_List.sensors[1].range = sensorValue;

  data_List.sensors[2].sensor_id = "Capteur 2";
  data_List.sensors[2].range = 1234;

  pub_sensor1.publish(&data_List);

  nh.spinOnce();
  delay(500);
}
