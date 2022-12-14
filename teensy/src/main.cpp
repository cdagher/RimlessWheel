#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <cassert> 

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...

// print stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


// #define USE_TEENSY_HW_SERIAL // for using a hardware serial port w/ ROS
#define MOTOR_SUBSCRIBER_NAME "/torso_command"
#define ODRIVE_SUBSCRIBER_NAME "/odrive_command"
#define ENCODER_PUBLISHER_NAME "/sensors"

#define MOTOR_VELOCITY_LIMIT 10.0 // This is in rotations per second
#define MOTOR_CURRENT_LIMIT  50.0 // amps


ros::NodeHandle nh;

void receiveJointState(const sensor_msgs::JointState &msg);
sensor_msgs::JointState motorStates; // comands for the motors
ros::Subscriber<sensor_msgs::JointState> motors(MOTOR_SUBSCRIBER_NAME, &receiveJointState);

void receiveODriveCommand(const sensor_msgs::Joy &msg);
sensor_msgs::Joy odriveCommand; // commands for clearing errors, rebooting, etc
ros::Subscriber<sensor_msgs::Joy> odriveCmd(ODRIVE_SUBSCRIBER_NAME, &receiveODriveCommand);

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
// #define TORQUE_CONTROL

const float m1 = 2.32f;    
const float m2 = 4.194f; 
const float I1 = 0.0784160f;
const float I2 = 0.0380256f;
const float mt = m1 + m2;
const float l1 = 0.26f;
const float l2 = 0.05f;
const float g  = 9.81f;
const float incline = 0.0f; 
const float k = 10.0f;
const float alpha = 360.0f/k/2.0f * M_PI/180.0f;
const float Kv = 0.13f;

const float samplingTime = 1.0f/FILTER_UPDATE_RATE_HZ;
float oldTorsoAngle = 0.0f;
float oldSpoke1Angle = M_PI - alpha;
float oldSpoke2Angle = M_PI - alpha;
float oldTorsoSpeed = 0.0f;
float oldSpokeSpeed = 0.0f;
float oldAppliedTorque = 0.0f;
uint32_t timestamp;
bool impactOccurredBefore = false;

void setup() {

  nh.initNode();
  nh.subscribe(motors);
  nh.subscribe(odriveCmd);
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
  odriveSerial.begin(115200);

  // TODO: figure out why this returns zero. Maybe the ODrive needs a FW update?
  odriveSerial << "r vbus_voltage\n";
  Serial << "Vbus voltage: " << ODrive.readVoltage() << '\n';

  Serial.println("Setting parameters...");

  // set the parameters for both motors
  for (int axis = 0; axis < 2; ++axis) {
    odriveSerial << "w axis" << axis << ".controller.config.vel_limit " << MOTOR_VELOCITY_LIMIT << '\n';
    odriveSerial << "w axis" << axis << ".motor.config.current_lim " << MOTOR_CURRENT_LIMIT << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  // calibrate the motors (this is required for them to run)
  calibrateMotor(0);
  calibrateMotor(1);

  //Keep track of time to sample the encoders and the IMU evenly
  timestamp = millis();

  Serial.println("Ready!");
}

void loop() { 

  if ((millis() - timestamp) <= (1000*samplingTime)) {
    return;
  }

  timestamp = millis();
  publishSensorStates();

  nh.spinOnce();
}

void receiveJointState(const sensor_msgs::JointState &msg) {

#if defined(TORQUE_CONTROL)
  // float torque = msg.effort[0];
  //  ODrive.SetCurrent(0, torque/Kv);
  //  ODrive.SetCurrent(1, torque/Kv);
 
  //  oldAppliedTorque = torque;  // TODO: set oldAppliedTorque to torque applied by the motor 
   //NOTE: not correct in velocity control
#else
  float velocity0 = msg.velocity[0];
  float velocity1 = msg.velocity[1];
  ODrive.SetVelocity(0, -1*velocity0*MOTOR_VELOCITY_LIMIT);
  ODrive.SetVelocity(1, velocity1*MOTOR_VELOCITY_LIMIT);
#endif
    
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

void receiveODriveCommand(const sensor_msgs::Joy &msg) {
  if (msg.buttons[0] == 1) {
    ODrive.SetVelocity(0, 0);
    ODrive.SetVelocity(1, 0);

    ODrive.clearErrors();
    delay(250);

    calibrateMotor(0);
    calibrateMotor(1);

  } else if (msg.buttons[3] == 1) {
    ODrive.SetVelocity(0, 0);
    ODrive.SetVelocity(1, 0);

    ODrive.reboot();
    delay(2000);

    calibrateMotor(0);
    calibrateMotor(1);
  }
}

float* cross(float x[3], float y[3]){

  static float crossProd[3];
  crossProd[0] = x[1]*y[2] - x[2]*y[1];
  crossProd[1] = x[2]*y[0] - x[0]*y[2];
  crossProd[2] = x[0]*y[1] - x[1]*y[0];

  return crossProd;

}

float alphaDynamics(float u, float theta, float phi, float thetadot, float phidot){

  float BCG[2] = {-u + m2*l1*l2*sinf(theta-phi)*powf(phidot, 2.0f) + g*mt*l1*sinf(theta-incline), 
                  u - m2*l1*l2*sinf(theta-phi)*powf(thetadot, 2.0f) - g*m2*l2*sinf(phi-incline)};
  float detM = (-I1*I2 - I1*powf(l2,2.0f)*m2 - I2*powf(l1, 2.0f)*mt + powf(cosf(theta-phi)*l1*l2*m2, 2.0f) - powf(l1*l2, 2.0f)*m2*mt);
  
  float phidotdot = 1.0f/detM*( (-cosf(theta-phi)*l1*l2*m2)*BCG[0] + (-I1 - powf(l1, 2.0f)*mt)*BCG[1]);
  
  return phidotdot;
}

float* comAcceleration(const sensors_event_t& accel, const sensors_event_t& gyro, float alpha_x){
  
  static float acc_COM[3]; 

  float imuToCOM[3] = {0.0f, 0.0f, 0.0f};
  float omega[3]    = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};
  float ap[3]       = {accel.acceleration.x, accel.acceleration.y, accel.acceleration.z};
  float alpha[3]    = {alpha_x, 0.0f, 0.0f}; //assumes the robot has no tolerance/play in the y and z direction

  auto omegaCrossR  = cross(omega, imuToCOM);
  auto alphaCrossR  = cross(alpha, imuToCOM);
  auto omegaCrossOmegaCrossR = cross(omega, omegaCrossR); 

  for (int i = 0; i < 3; ++i){
    acc_COM[i] = ap[i] - alphaCrossR[i] - omegaCrossOmegaCrossR[i];
  }

  return acc_COM;
}

bool impactDetected(){
  return false;
}

float* impactMap(float phi, float thetadot, float phidot){

  float det = I1*I2 + I1*m2*powf(l2, 2.0f) + I2*mt*powf(l1, 2.0f) + m2*powf(l1*l2, 2.0f)*(m1 + m2*powf(sinf(alpha - phi),2.0f));

  float a1 = 1.0f/det * ((I1*I2 + I1*m2*powf(l2, 2.0f)) + 
          (I2*mt*powf(l1, 2.0f) + m2*powf(l1*l2, 2.0f)*(m1 + 0.5f* m2))*cosf(2.0f*alpha) -
          0.5*powf(m2*l1*l2, 2.0f)*cosf(2.0f*phi));

  float a2 = 1.0f/det *(m2*l1*l2*(I1*(cosf(alpha - phi) - cosf(alpha + phi)) + 
          mt*powf(l1, 2.0f)*(cosf(2.0f*alpha)*cosf(alpha - phi) - cosf(alpha + phi))) );

  static float vel[2] = {a1*thetadot, a2*thetadot+phidot};
  return vel;
}

bool encoderSymmetryCheck(float encPos0, float encPos1){
  return abs(encPos0 - encPos1) < 0.001;
}

void publishSensorStates() {

  //All angles are given in radians.

  float encPos0 = 0.0;
  float encPos1 = 0.0;
  float encVel0 = 0.0;
  float encVel1 = 0.0;
  float gx, gy, gz;

  //Read encoder from ODrive
  // encPos0 = ODrive.GetPosition(0);
  // encPos1 = ODrive.GetPosition(1);

  assert(encoderSymmetryCheck(encPos0, encPos1));

  //TODO: test GetVelocity
  // encVel0 = ODrive.GetVelocity(0);
  // encVel1 = ODrive.GetVelocity(1);

  // float encVel0 = (encPos0 - oldSpoke1Angle)/samplingTime;
  // float encVel1 = (encPos1 - oldSpoke2Angle)/samplingTime;

  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  //convert the linear acceleration of the IMU to the acceleration of the center of mass
  // Compute the angular acceleration from the dynamics inorder to shift the linear acceleration at the COM
  //find shifted linear acceleration
  float theta, phi, thetadot, phidot;
  if (!impactDetected()){
    theta     = encPos0;
    phi       = mag.magnetic.x;
    thetadot  = gyro.gyro.x;
    phidot    = encVel0;
    impactOccurredBefore = false;
  }
  else {
    if (impactOccurredBefore){
        theta     = oldSpoke1Angle;
        phi       = oldTorsoAngle;
        thetadot  = oldSpokeSpeed;
        phidot    = oldTorsoSpeed;
      }
    else{
        theta       = oldSpoke1Angle;
        phi         = oldTorsoAngle;
        auto vel    = impactMap(phi, oldSpokeSpeed, oldTorsoSpeed);
        thetadot    = vel[0];
        phidot      = vel[1];
        impactOccurredBefore = true;
      }
  }
  
  float alpha_x = alphaDynamics(oldAppliedTorque, theta, phi, thetadot, phidot);
  auto acc_COM = comAcceleration(accel, gyro, alpha_x);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                acc_COM[0], acc_COM[1], acc_COM[2], 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  float torsoRoll    = filter.getRoll() * 1.0f/SENSORS_RADS_TO_DPS;
  float torsoHeading = filter.getYaw() * 1.0f/SENSORS_RADS_TO_DPS;
  float torsoPitch   = filter.getPitch() * 1.0f/SENSORS_RADS_TO_DPS;

  //Update torso angular velocity
  float torsoOmega = gyro.gyro.x;
  // float torsoOmega =  (torsoRoll - oldTorsoAngle)/samplingTime;
  oldTorsoAngle = torsoRoll;
  oldTorsoSpeed = torsoOmega;
  oldSpoke1Angle = encPos0;
  oldSpoke2Angle = encPos1; 
  oldSpokeSpeed = encVel0;

#if defined(AHRS_DEBUG_OUTPUT)
  Serial.print("Orientation: ");
  Serial.print(torsoHeading*SENSORS_RADS_TO_DPS);
  Serial.print(", ");
  Serial.print(torsoPitch*SENSORS_RADS_TO_DPS);
  Serial.print(", ");
  Serial.println(torsoRoll*SENSORS_RADS_TO_DPS);
#endif

  sensorStates.position_length = 3;
  sensorStates.velocity_length = 3;
  float sensorPosition[3] = 
      {
      torsoRoll, encPos0, encPos1,
      };
  float sensorVelocity[3] = 
      {
      torsoOmega, encVel0, encVel1
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
