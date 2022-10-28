#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...

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

void publishSensorStates();
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

Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
#define AHRS_DEBUG_OUTPUT

const float samplingTime = 1.0/FILTER_UPDATE_RATE_HZ;
float oldTorsoAngle = -999;
float oldSpoke1Angle = -999;
float oldSpoke2Angle = -999;
uint32_t timestamp;

void setup() {

  nh.initNode();
  nh.subscribe(motors);
  nh.advertise(sensors);

  // Serial output over USB
  Serial.begin(115200);
  while (!Serial) ; // wait for USB connection

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);

  Wire.setClock(400000); // 400KHz
  
  // ODrive uses 115200 as the baudrate
  // odriveSerial.begin(115200);

  // TODO: figure out why this returns zero. Maybe the ODrive needs a FW update?
  odriveSerial << "r vbus_voltage\n";
  Serial << "Vbus voltage: " << ODrive.readFloat() << '\n';

  Serial.println("Setting parameters...");

  // set the parameters for both motors
  // for (int axis = 0; axis < 2; ++axis) {
  //   odriveSerial << "w axis" << axis << ".controller.config.vel_limit " << MOTOR_VELOCITY_LIMIT << '\n';
  //   odriveSerial << "w axis" << axis << ".motor.config.current_lim " << MOTOR_CURRENT_LIMIT << '\n';
  //   // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  // }

  // calibrate the motors
  // calibrateMotor(0);
  // calibrateMotor(1);

  //Keep track of time to sample the encoders and the IMU evenly
  timestamp = millis();

  Serial.println("Ready!");
}

void loop() { 

  if ((millis() - timestamp) < (1000*samplingTime)) {
    return;
  }

  timestamp = millis();
  publishSensorStates();
  // subscribeCommandMotorStates();

  nh.spinOnce();
}

void receiveJointState(const sensor_msgs::JointState &msg) {

    // TODO: actually write this method
    // received joint state should be something like:
    // { torque0, velocity0, torque1, velocity1 }
    // where the 1st two values are for motor 0 and the 2nd two are for motor 1.
    // One of these values per motor should be zero, and will be ignored
    // The topic name for the torque command is "/torso_command"
    
    // IF ALL VALUES RECEIVED ARE ZERO, STOP MOTORS
    // If one motor receives all zeros and the other doesn't, stop the motor with zeros
    return;
}

void publishSensorStates() {

  float encPos0 = 0.0;
  float encPos1 = 0.0;
  float gx, gy, gz;

  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);
  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  float torsoRoll    = filter.getRoll();
  float torsoHeading = filter.getYaw();
  float torsoPitch   = filter.getPitch();

  //Update torso angular velocity
  float torsoOmega = gx * 1.0 / SENSORS_RADS_TO_DPS;
  // float torsoOmega =  (torsoRoll - oldTorsoAngle)/samplingTime;
  // oldTorsoAngle = torsoRoll;

  //Read encoder from ODrive
  // encPos0 = ODrive.GetPosition(0);
  // encPos1 = ODrive.GetPosition(1);

  float spoke1Omega = (encPos0 - oldSpoke1Angle)/samplingTime;
  oldSpoke1Angle    = encPos0;
  float spoke2Omega = (encPos1 - oldSpoke2Angle)/samplingTime;
  oldSpoke2Angle    = encPos1; 

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Orientation: ");
  Serial.print(torsoHeading);
  Serial.print(", ");
  Serial.print(torsoPitch);
  Serial.print(", ");
  Serial.println(torsoRoll);
#endif

  sensorStates.position_length = 3;
  sensorStates.velocity_length = 3;
  float sensorPosition[3] = 
      {
      torsoRoll, encPos0, encPos1,
      };
  float sensorVelocity[3] = 
      {
      torsoOmega, spoke1Omega, spoke2Omega
    };

  sensorStates.position = sensorPosition;
  sensorStates.velocity = sensorVelocity;
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
