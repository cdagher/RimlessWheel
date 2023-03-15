#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <cassert> 
#include <filters.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"  // can adjust to LSM6DS33, LSM6DS3U, LSM6DSOX...

// print stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// #define TO_STREAM(stream,variable) (stream) <<#variable": "<<(variable) // see https://cplusplus.com/forum/beginner/11252/


// #define USE_TEENSY_HW_SERIAL // for using a hardware serial port w/ ROS
#define MOTOR_SUBSCRIBER_NAME "/torso_command"
#define ODRIVE_SUBSCRIBER_NAME "/odrive_command"
#define ENCODER_PUBLISHER_NAME "/sensors"
#define ODRIVE_ERROR_PUBLISHER_NAME "/odrive_errors"

#define MOTOR_VELOCITY_LIMIT 50.0 // radians per second? Maybe rotations per second?
#define MOTOR_CURRENT_LIMIT  20.0 // amps


ros::NodeHandle nh;

void receiveJointState(const sensor_msgs::JointState &msg);
sensor_msgs::JointState motorStates; // comands for the motors
ros::Subscriber<sensor_msgs::JointState> motors(MOTOR_SUBSCRIBER_NAME, &receiveJointState);

void receiveODriveCommand(const sensor_msgs::Joy &msg);
sensor_msgs::Joy odriveCommand; // commands for clearing errors, rebooting, etc
ros::Subscriber<sensor_msgs::Joy> odriveCmd(ODRIVE_SUBSCRIBER_NAME, &receiveODriveCommand);

void publishSensorStates(const float* torsoStates, const float* spokeStates );
sensor_msgs::JointState sensorStates; // feedback of the encoder positions
ros::Publisher sensors(ENCODER_PUBLISHER_NAME, &sensorStates);

void publishErrorState();
std_msgs::Int64MultiArray errorStates; // error states for the ODrive
ros::Publisher odriveErrors(ODRIVE_ERROR_PUBLISHER_NAME, &errorStates);

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX (GPIO1)
// pin 1: TX - connect to ODrive RX (GPIO2)
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
HardwareSerial &odriveSerial = Serial1;

// ODrive object
ODriveArduino ODrive(odriveSerial);
// E-stop pins
int estop_in = 3;

bool estop();
void brake();
void calibrateMotor(bool motor);
int64_t readErrors();
int64_t readODriveErrors();
int64_t readMotorErrors(int motor_number);
int64_t readAxisErrors(int axis_number);
int64_t readEncoderErrors(int encoder_number);
int64_t readControllerErrors(int controller_number);
float* readEncoder(float* torsoStates);
float* readIMU();
void commandTorque(int axis, float torque);
void computeTorque(const float* torsoStates, const float* spokeStates);

Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
#define AHRS_DEBUG_OUTPUT
#define TORQUE_CONTROL
#define ODRIVE_CONNECTED

const float m1 = 1.13f;    
const float m2 = 3.385f; 
const float l1 = 0.3f;
const float l2 = 0.06f;
const float I1 = 0.0885f/2.0f;
const float I2 = m2*l2*l2/3.0f;
const float mt = m1 + m2;
const float W = 0.026f;
const float g  = 9.81f;
const float incline = 0.0f; 
const float k = 10.0f;
const float alpha = 360.0f/k/2.0f * M_PI/180.0f;
const float Kv = 0.13f;
const float gearRatio = 1.0f/6.0f;

const float samplingTime = 1.0f/FILTER_UPDATE_RATE_HZ;
float oldTorsoOmega = 0.0f;
float oldSpoke1Angle = alpha;
float oldSpoke2Angle = alpha;
float oldSpokeSpeed = 0.0f;
float torque0 = 0.0;
float torque1 = 0.0;
float enc0Offset = 0.0;
float enc1Offset = 0.0;
float yawOffset = 0.0;

uint32_t timestamp;
bool impactOccurredBefore = false;

IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)
Filter lpf = Filter(30.0, samplingTime, order);

void setup() {

  nh.initNode();
  nh.subscribe(motors);
  nh.subscribe(odriveCmd);
  nh.advertise(sensors);
  nh.advertise(odriveErrors);

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
  
  pinMode(estop_in, INPUT_PULLDOWN);

  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);

  Wire.setClock(400000); // 400KHz
  
  #if defined(ODRIVE_CONNECTED)
    // ODrive uses 115200 as the baudrate
    odriveSerial.begin(115200);

    // odriveSerial << "sr" << "\n";
    // delay(5000);
    
    // TODO: figure out why this returns zero. Maybe the ODrive needs a FW update?
    odriveSerial << "r vbus_voltage\n";
    Serial << "Vbus voltage: " << ODrive.readFloat() << '\n';

    Serial.println("Setting parameters...");

    // set the parameters for both motors
    for (int axis = 0; axis < 2; ++axis) {
      odriveSerial << "w axis" << axis << ".error " << 0 << '\n';
      odriveSerial << "w axis" << axis << ".controller.config.vel_limit " << MOTOR_VELOCITY_LIMIT << '\n';
      odriveSerial << "w axis" << axis << ".motor.config.current_lim " << MOTOR_CURRENT_LIMIT << '\n';
      // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
    }

    // calibrate the motors
    calibrateMotor(0);
    calibrateMotor(1);

    delay(5000);

    #if defined(TORQUE_CONTROL)
      for (int axis = 0; axis < 2; ++axis) {
        odriveSerial << "w axis" << axis << ".controller.config.control_mode " << CONTROL_MODE_TORQUE_CONTROL << '\n';
        odriveSerial << "w axis" << axis << ".motor.config.torque_constant " << 8.23 / 210.0 << '\n';
        odriveSerial << "w axis" << axis << ".motor.controller.enable_torque_mode_vel_limit = False" << '\n';
      }
    #endif

    for (int axis = 0; axis < 2; ++axis) {
        odriveSerial << "w axis" << axis << ".motor.config.startup_closed_loop_control = False" << '\n';
    }

    auto torsoStates = readIMU();
    auto spokeStates = readEncoder(torsoStates);
  
    delay(250);
    enc0Offset = spokeStates[0];
    enc1Offset = spokeStates[1];
    yawOffset = torsoStates[2];


  #endif

  //Keep track of time to sample the encoders and the IMU evenly
  timestamp = millis();

  Serial.println("Ready!");
}

void loop() { 

  if ((millis() - timestamp) >= (1000*samplingTime)) {

    timestamp = millis();
    
    auto torsoStates = readIMU();
    auto spokeStates = readEncoder(torsoStates);
    publishSensorStates(torsoStates, spokeStates);

    #if defined(ODRIVE_CONNECTED)
      if (readErrors() != 0) {
        // TODO: clear non-critical errors
        odriveErrors.publish(&errorStates);
      }
    #endif

    computeTorque(torsoStates, spokeStates);
  }
  nh.spinOnce();

}

void computeTorque(const float* torsoStates, const float* spokeStates){
  if (estop()){
      brake();
      while (estop()) {

        commandTorque(0, 0);
        commandTorque(1, 0);

        auto getTorsoStates = readIMU();
        auto getSpokeStates = readEncoder(getTorsoStates);

        publishSensorStates(getTorsoStates, getSpokeStates);
      }

      //When the encoder wraps, and you switch the Estop off, it starts from configurations not visited by the training. So, unwrap it. 

      enc0Offset = 0.0f;
      enc1Offset = 0.0f;
      auto getTorsoStates = readIMU();
      auto getSpokeStates = readEncoder(getTorsoStates);
      enc0Offset = getSpokeStates[0];
      enc1Offset = getSpokeStates[1];
    }
  else{

    commandTorque(0, -1.0f*torque0);
    commandTorque(1, 1.0f*torque1);
  }
}

void receiveJointState(const sensor_msgs::JointState &msg) {

  #if defined(TORQUE_CONTROL)

      ///////////// for neural net //////////////

      torque0 = msg.effort[0];
      // torque1 = msg.effort[1];
      torque1 = torque0;
      Serial.print("Received torque command: ");
      Serial.print(torque0);
      Serial.print(torque1);

      ///////////// for joystick ////////////////////
      // torque0 = msg.velocity[0];
      // torque1 = torque0;

    #else
    #if defined(ODRIVE_CONNECTED)
      float velocity0 = msg.velocity[0];
      float velocity1 = msg.velocity[1];
      ODrive.SetVelocity(0, -1*velocity0*MOTOR_VELOCITY_LIMIT);
      ODrive.SetVelocity(1, velocity1*MOTOR_VELOCITY_LIMIT);

      #endif 
    #endif
  
  return;
}

void receiveODriveCommand(const sensor_msgs::Joy &msg) {
  if (msg.buttons[0] == 1) {
    ODrive.SetVelocity(0, 0);
    ODrive.SetVelocity(1, 0);

    odriveSerial << "sc" << "\n";
    delay(250);

    calibrateMotor(0);
    calibrateMotor(1);

  } else if (msg.buttons[3] == 1) {
    ODrive.SetVelocity(0, 0);
    ODrive.SetVelocity(1, 0);

    odriveSerial << "sr" << "\n";
    delay(2000);

    calibrateMotor(0);
    calibrateMotor(1);
  }
}

void commandTorque(int axis, float torque){
  odriveSerial << "w axis" << axis << ".controller.input_torque " << torque << '\n';
}

bool estop(){
  if (digitalRead(estop_in) == LOW){
    return true;
  }
  else{
    return false;
  }
}

void brake(){
  commandTorque(0, 0);
  commandTorque(1, 0);
}

float* cross(float x[3], float y[3]){

  static float crossProd[3];
  crossProd[0] = x[1]*y[2] - x[2]*y[1];
  crossProd[1] = x[2]*y[0] - x[0]*y[2];
  crossProd[2] = x[0]*y[1] - x[1]*y[0];

  return crossProd;

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

float* readEncoder(float* torsoStates){

  static float spokeStates[4];
  spokeStates[0] = -abs(ODrive.GetPosition(0))*2.0f*M_PI*gearRatio - enc0Offset;
  spokeStates[1] = -abs(ODrive.GetPosition(1))*2.0f*M_PI*gearRatio - enc1Offset;
  // spokeStates[2] = ODrive.GetVelocity(0);
  // spokeStates[3] = ODrive.GetVelocity(1);

  spokeStates[2] = lpf.filterIn((spokeStates[0] - oldSpoke1Angle)/samplingTime);
  spokeStates[3] = lpf.filterIn((spokeStates[1] - oldSpoke2Angle)/samplingTime);
  oldSpoke1Angle = spokeStates[0] ;
  oldSpoke2Angle = spokeStates[1]; 
  oldSpokeSpeed = spokeStates[2];

  return spokeStates;
}

float* readIMU(){

  static float torsoStates[3]; 

  //All angles are given in radians.
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);


  // Compute the angular acceleration from the dynamics inorder to shift the linear acceleration at the COM
  //find shifted linear acceleration
  
  float alpha_x = (gyro.gyro.x - (-1.0f*oldTorsoOmega))/samplingTime;
  auto acc_COM = comAcceleration(accel, gyro, alpha_x);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  float gx, gy, gz;
  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                acc_COM[0], acc_COM[1], acc_COM[2], 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  torsoStates[0] = -filter.getRoll() * 1.0f/SENSORS_RADS_TO_DPS;
  torsoStates[1] = -gyro.gyro.x;
  torsoStates[2] = filter.getYaw() * 1.0f/SENSORS_RADS_TO_DPS - yawOffset;
  // torsoStates[2]   = filter.getPitch() * 1.0f/SENSORS_RADS_TO_DPS - yawOffset;
  oldTorsoOmega = torsoStates[1];

  return torsoStates;
}

void publishSensorStates(const float* torsoStates, const float* spokeStates) {

  float encPos0 = spokeStates[0];
  float encPos1 = spokeStates[1];
  float encVel0 = spokeStates[2];
  float encVel1 = spokeStates[3];
  float torsoRoll = torsoStates[0];
  float torsoOmega = torsoStates[1];
  float yaw = torsoStates[2];

  #if defined(AHRS_DEBUG_OUTPUT)
    Serial.print("Sensor: ");
    Serial.print(torsoRoll);
    Serial.print(", ");
    Serial.print(encPos0);
    Serial.print(", ");
    Serial.print(encPos1);
    Serial.print(", ");
    // Serial.print(encPos0 - (-torsoRoll));
    // Serial.print(", ");
    // Serial.print(encPos1 - (torsoRoll));
    // Serial.print(", ");
    Serial.println(yaw);
    Serial.print("Angular velocities: ");
    Serial.println(torsoOmega);
    Serial.print(", ");
    Serial.print(encVel0);
    Serial.print(", ");
    Serial.println(encVel1);
  #endif

  sensorStates.header = std_msgs::Header();
  sensorStates.header.stamp = nh.now();
  sensorStates.position_length = 4;
  sensorStates.velocity_length = 3;

  float sensorPosition[4] = 
      {
      torsoRoll, encPos0, encPos1, yaw
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

int64_t readErrors() {
  int64_t errors[9];
  errorStates.data_length = 9;

  errors[0] = readODriveErrors();

  errors[1] = readMotorErrors(0);
  errors[2] = readMotorErrors(1);

  errors[3] = readAxisErrors(0);
  errors[4] = readAxisErrors(1);

  errors[5] = readEncoderErrors(0);
  errors[6] = readEncoderErrors(1);

  errors[7] = readControllerErrors(0);
  errors[8] = readControllerErrors(1);

  errorStates.data = errors;

  int64_t ret = 0;
  for (int i=0; i<9; i++) {
    ret |= errors[i] != 0;
  }
  
  return ret;
}

int64_t readODriveErrors() {
    odriveSerial << "error" << "\n";
    return ODrive.readlong();
}

int64_t readMotorErrors(int motor_number) {
    odriveSerial << "r axis" << motor_number << ".motor.error" << "\n";
    return ODrive.readlong();
}

int64_t readAxisErrors(int axis_number) {
    odriveSerial << "r axis" << axis_number << ".error" << "\n";
    return ODrive.readlong();
}

int64_t readEncoderErrors(int encoder_number) {
    odriveSerial << "r axis" << encoder_number << ".encoder.error" << "\n";
    return ODrive.readlong();
}

int64_t readControllerErrors(int controller_number) {
    odriveSerial << "r axis" << controller_number << ".controller.error" << "\n";
    return ODrive.readlong();
}
