#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_LSM6DSOX.h>
#include "Adafruit_Sensor_Calibration.h"

// print stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


// #define USE_TEENSY_HW_SERIAL // for using a hardware serial port w/ ROS
#define MOTOR_SUBSCRIBER_NAME "/motors"
#define ENCODER_PUBLISHER_NAME "/sensors"

ros::NodeHandle nh;
sensor_msgs::JointState sensorStates; // feedback of the encoder positions
void populatePublishSensorStates();
ros::Publisher sensors(ENCODER_PUBLISHER_NAME, &sensorStates);


// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX (GPIO1)
// pin 1: TX - connect to ODrive RX (GPIO2)
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy


Adafruit_LSM6DSOX imu;
data_rate imuDataRate = LSM6DS_RATE_1_66K_HZ;
accel_range imuAccelRange = LSM6DS_ACCEL_RANGE_4_G;
gyro_range imuGyroRange = LSM6DS_GYRO_RANGE_500_DPS;

void setup() {

  nh.initNode();
  nh.advertise(sensors);

  if (!imu.begin_I2C()) {
    while (1) ; // hang b/c we need the IMU
  }
  imu.setAccelDataRate(imuDataRate);
  imu.setGyroDataRate(imuDataRate);
  imu.setAccelRange(imuAccelRange);
  imu.setGyroRange(imuGyroRange);

  // Serial output over USB
  Serial.begin(115200);
  while (!Serial) ; // wait for USB connection
  Serial.println("Ready!");
}

void loop() {    

    populatePublishSensorStates();
    nh.spinOnce();
}

void populatePublishSensorStates() {

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp);
    sensorStates.position_length = 6;

    float sensorStateVals[6] = 
        {
        accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, // accelerometer values
        gyro.gyro.x, gyro.gyro.y, gyro.gyro.z // gyroscope values
        };
    // Serial.println(accel.acceleration.x);  

    sensorStates.position = sensorStateVals;
    sensors.publish(&sensorStates);

}
