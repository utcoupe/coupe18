#define Address1  0x00
#define Address2  0x00

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h> 
#include <geometry_msgs/Vector3.h>

//#include "processing_belt_interpreter/RangeList.h"

#include <Arduino.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"


MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

bool blinkState = false;

ros::NodeHandle  nh;

std_msgs::Float32 value;
std_msgs::String str_msg;
geometry_msgs::Vector3 sensorValues;
//processing_belt_interpreter::RangeList data_List;

ros::Publisher chatter("chatter", &str_msg);

// List of sensor publishers
ros::Publisher pub_sensor1("sensors/belt", &sensorValues);

//Sensor addresses
int sensorAddress1 = Address1;
int sensorAddress2 = Address2;

//Sensor data variable
int sensorPin = 11;

char hello[10] = "Salut"; //accelgyro.testConnection();

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(11, INPUT);

  Wire.begin();        // join i2c bus (address optional for master)

  accelgyro.initialize();
    
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
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  sensorValues.x = ax;
  sensorValues.y = ay;
  sensorValues.z = az;
  //sensorValues[3] = gx;
  //sensorValues[4] = gy;
  //sensorValues[5] = gz;
  pub_sensor1.publish(&sensorValues);

  blinkState = !blinkState;
  digitalWrite(13, blinkState);
  
  nh.spinOnce();
  delay(500);
}
