#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <Adafruit_LSM6DSOX.h>


// print stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


// #define USE_TEENSY_HW_SERIAL // for using a hardware serial port w/ ROS
#define MOTOR_SUBSCRIBER_NAME "/motors"
#define ENCODER_PUBLISHER_NAME "/sensors"

#define MOTOR_VELOCITY_LIMIT 10.0 // radians per second? Maybe rotations per second?
#define MOTOR_CURRENT_LIMIT  20.0 // amps


ros::NodeHandle nh;

void receiveJointState(const sensor_msgs::JointState &msg);
sensor_msgs::JointState motorStates; // comands for the motors
ros::Subscriber<sensor_msgs::JointState> motors(MOTOR_SUBSCRIBER_NAME, &receiveJointState);

void populatePublishSensorStates();
sensor_msgs::JointState sensorStates; // feedback of the encoder positions
ros::Publisher sensors(ENCODER_PUBLISHER_NAME, &sensorStates);


// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX (GPIO1)
// pin 1: TX - connect to ODrive RX (GPIO2)
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
HardwareSerial &odriveSerial = Serial1;

// ODrive object
ODriveArduino ODrive(odriveSerial);
void calibrateMotor(bool motor);

Adafruit_LSM6DSOX imu;
data_rate imuDataRate = LSM6DS_RATE_1_66K_HZ;
accel_range imuAccelRange = LSM6DS_ACCEL_RANGE_4_G;
gyro_range imuGyroRange = LSM6DS_GYRO_RANGE_500_DPS;

void setup() {

  nh.initNode();
  nh.subscribe(motors);
  nh.advertise(sensors);

  if (!imu.begin_I2C()) {
    while (1) ; // hang b/c we need the IMU
  }
  imu.setAccelDataRate(imuDataRate);
  imu.setGyroDataRate(imuDataRate);
  imu.setAccelRange(imuAccelRange);
  imu.setGyroRange(imuGyroRange);


  // ODrive uses 115200 as the baudrate
  odriveSerial.begin(115200);

  // Serial output over USB
  Serial.begin(115200);
  while (!Serial) ; // wait for USB connection

  // TODO: figure out why this returns zero. Maybe the ODrive needs a FW update?
  odriveSerial << "r vbus_voltage\n";
  Serial << "Vbus voltage: " << ODrive.readFloat() << '\n';

  Serial.println("Setting parameters...");

  // set the parameters for both motors
  for (int axis = 0; axis < 2; ++axis) {
    odriveSerial << "w axis" << axis << ".controller.config.vel_limit " << MOTOR_VELOCITY_LIMIT << '\n';
    odriveSerial << "w axis" << axis << ".motor.config.current_lim " << MOTOR_CURRENT_LIMIT << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  // calibrate the motors
  calibrateMotor(0);
  calibrateMotor(1);

  Serial.println("Ready!");
}

void loop() {    
    populatePublishSensorStates();
    nh.spinOnce();
}

void receiveJointState(const sensor_msgs::JointState &msg) {
    // TODO: actually write this method
    // received joint state should be something like:
    // { torque0, velocity0, torque1, velocity1 }
    // where the 1st two values are for motor 0 and the 2nd two are for motor 1.
    // One of these values per motor should be zero, and will be ignored

    // IF ALL VALUES RECEIVED ARE ZERO, STOP MOTORS
    // If one motor receives all zeros and the other doesn't, stop the motor with zeros
    return;
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


void calibrateMotor(bool motornum) {
  char c = motornum+'0';
  int requested_state;

  requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!ODrive.run_state(motornum, requested_state, true)) return;

  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!ODrive.run_state(motornum, requested_state, true, 25.0f)) return;

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!ODrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
}
