#ifndef ODriveHelper_h
#define ODriveHelper_h

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <ODriveEnums.h>

#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

class ODriveHelper {
public:
    ODriveHelper(HardwareSerial& serial);

    void begin(uint64_t baudRate);

    // Commands
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    void SetCurrent(int motor_number, float current);
    void TrapezoidalMove(int motor_number, float position);
    void calibrateMotor(bool motor_number);
    void setVelocityLimit(bool axis, float velocity_limit);
    void setCurrentLimit(bool axis, float current_limit);
    // Getters
    float GetVelocity(int motor_number);
    float GetPosition(int motor_number);
    float getBusVoltage();
    // Errors
    int32_t readErrors(std_msgs::Int32MultiArray& errorStates);
    int32_t readODriveErrors();
    int32_t readMotorErrors(int motor_number);
    int32_t readAxisErrors(int axis_number);
    int32_t readEncoderErrors(int encoder_number);
    int32_t readControllerErrors(int controller_number);
    // State helper
    bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f);

private:

    HardwareSerial& _serial;
    ODriveArduino _ODrive;

};

#endif