#include "ODriveHelper.h"

    // Print with stream operator
    template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
    template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

    ODriveHelper::ODriveHelper(HardwareSerial& serial)
        : _serial(serial), _ODrive(serial) {}

    void ODriveHelper::begin(uint64_t baudRate) {
        _serial.begin(baudRate);
    }

    // Commands
    void ODriveHelper::SetPosition(int motor_number, float position) {
        _ODrive.SetPosition(motor_number, position);
    }

    void ODriveHelper::SetPosition(int motor_number, float position, float velocity_feedforward) {
        _ODrive.SetPosition(motor_number, position, velocity_feedforward);
    }

    void ODriveHelper::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
        _ODrive.SetPosition(motor_number, position, velocity_feedforward, current_feedforward);
    }

    void ODriveHelper::SetVelocity(int motor_number, float velocity) {
        _ODrive.SetVelocity(motor_number, velocity);
    }

    void ODriveHelper::SetVelocity(int motor_number, float velocity, float current_feedforward) {
        _ODrive.SetVelocity(motor_number, velocity, current_feedforward);
    }

    void ODriveHelper::SetCurrent(int motor_number, float current) {
        _ODrive.SetCurrent(motor_number, current);
    }

    void ODriveHelper::TrapezoidalMove(int motor_number, float position) {
        _ODrive.TrapezoidalMove(motor_number, position);
    }

    void ODriveHelper::calibrateMotor(bool motor_number) {
        char c = motor_number+'0';
        int requested_state;

        requested_state = AXIS_STATE_MOTOR_CALIBRATION;
        Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
        if(!run_state(motor_number, requested_state, true)) return;

        requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
        Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
        if(!run_state(motor_number, requested_state, true, 25.0f)) return;

        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
        if(!run_state(motor_number, requested_state, false /*don't wait*/)) return;
    }

    void ODriveHelper::setVelocityLimit(bool axis, float velocity_limit) {
        _serial << "w axis" << axis << ".controller.config.vel_limit " << velocity_limit << '\n';
        // This ends up writing something like "w axis0.motor.config.velocity_limit_lim 10.0\n"
    }

    void ODriveHelper::setCurrentLimit(bool axis, float current_limit) {
        _serial << "w axis" << axis << ".motor.config.current_lim " << current_limit << '\n';
        // This ends up writing something like "w axis0.motor.config.current_limit 10.0\n"
    }

    // Getters
    float ODriveHelper::GetVelocity(int motor_number) {
        return _ODrive.GetVelocity(motor_number);
    }

    float ODriveHelper::GetPosition(int motor_number) {
        return _ODrive.GetPosition(motor_number);
    }
    
    float ODriveHelper::getBusVoltage() {
        _serial << "r vbus_voltage\n";
        return _ODrive.readFloat();
    }

    // Errors
    int32_t ODriveHelper::readErrors(std_msgs::Int32MultiArray& errorStates) {
    int32_t errors[9];
    errorStates.data_length = 9; // TODO: mutex

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

    int32_t ret = 0;
    for (int i=0; i<9; i++) {
        ret |= errors[i] != 0;
    }
    
    return ret;
    }

    int32_t ODriveHelper::readODriveErrors() {
        _serial << "error" << "\n";
        return _ODrive.readInt();
    }

    int32_t ODriveHelper::readMotorErrors(int motor_number) {
        _serial << "r axis" << motor_number << ".motor.error" << "\n";
        return _ODrive.readInt();
    }

    int32_t ODriveHelper::readAxisErrors(int axis_number) {
        _serial << "r axis" << axis_number << ".error" << "\n";
        return _ODrive.readInt();
    }

    int32_t ODriveHelper::readEncoderErrors(int encoder_number) {
        _serial << "r axis" << encoder_number << ".encoder.error" << "\n";
        return _ODrive.readInt();
    }

    int32_t ODriveHelper::readControllerErrors(int controller_number) {
        _serial << "r axis" << controller_number << ".controller.error" << "\n";
        return _ODrive.readInt();
    }

    // State helper
    bool ODriveHelper::run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f) {
        return _ODrive.run_state(axis, requested_state, wait_for_idle, timeout);
    }

    // ODrive functions
    void ODriveHelper::clearErrors() {
        _serial << "sc" << "\n";
        delay(50);
    }
    
    void ODriveHelper::reboot() {
        _serial << "sr" << "\n";
        delay(100);
    }
