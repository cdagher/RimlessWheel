#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>

// print stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


#define MOTOR_VELOCITY_LIMIT 10.0 // radians per second?
#define MOTOR_CURRENT_LIMIT  20.0 // amps

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX (GPIO1)
// pin 1: TX - connect to ODrive RX (GPIO2)
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
HardwareSerial &odriveSerial = Serial1;

// ODrive object
ODriveArduino ODrive(odriveSerial);


void calibrateMotor(bool motor);

void setup() {
    // ODrive uses 115200 as the baudrate
  odriveSerial.begin(115200);

  // Serial output over USB
  Serial.begin(115200);
  while (!Serial) ; // wait for USB connection

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
