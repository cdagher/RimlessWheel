#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS.h>
#include <HardwareSerial.h>
#include <../lib/ODrive/Arduino/ODriveArduino/ODriveArduino.h>


// print stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX (GPIO1)
// pin 1: TX - connect to ODrive RX (GPIO2)
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
HardwareSerial &odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);


void calibrate(bool motor);
void sinusoidalMove(bool motor);
void sinusoidalMove();

void testSetup() {
  // ODrive uses 115200 as the baudrate
  odrive_serial.begin(115200);

  // Serial output over USB
  Serial.begin(115200);
  while (!Serial) ; // wait for USB connection

  Serial.println("ODrive Motor Testing");

  odrive_serial << "r vbus_voltage\n";
  Serial << "Vbus voltage: " << odrive.readFloat() << '\n';

  Serial.println("Setting parameters...");

  // set the parameters for both motors
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  // calibrate the motors
  calibrate(0);
  calibrate(1);

  Serial.println("Ready!");

}

void testLoop() {

  while (!Serial) ; // as an e-stop kind of thing, but not very responsive

  sinusoidalMove(0);
  delay(100);
  sinusoidalMove(1);
  delay(500);

  sinusoidalMove();
  delay(1000);
}

void calibrate(bool motornum) {
  char c = motornum+'0';
  int requested_state;

  requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, true)) return;

  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
}

void sinusoidalMove(bool motor) {
  for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
    float pos = 2.0f * sin(ph);
    odrive.SetPosition(motor, pos);
    delay(5);
  }
}

void sinusoidalMove() {
  for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
    float pos_m0 = 2.0f * cos(ph);
    float pos_m1 = 2.0f * sin(ph);
    odrive.SetPosition(0, pos_m0);
    odrive.SetPosition(1, pos_m1);
    delay(5);
  }
}
