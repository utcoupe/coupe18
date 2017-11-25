#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

#include <Arduino.h>

ros::NodeHandle  nh;

std_msgs::Float32 value;
std_msgs::String str_msg;

ros::Publisher chatter("chatter", &str_msg);

// List of sensor publishers
ros::Publisher pub_sensor1("sensor1", &value);


//Sensor addresses
int sensorAddress = 0x00;

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

  // Code to get sensors' data
  sensorValue = analogRead(sensorPin);

  sensorValue = (sensorValue * 5 ) / 1023;
  value.data = 0.123/sensorValue;
  pub_sensor1.publish(&value);

  nh.spinOnce();
  delay(500);
}
