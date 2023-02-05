#include "rosHelper.h"

    rosHelper::rosHelper(
        sensor_msgs::JointState& motorStates,
        sensor_msgs::Joy& odriveCommand,
        sensor_msgs::JointState& sensorStates, 
        std_msgs::Int32MultiArray& errorStates)
        : _motorStates(motorStates),
          _odriveCommand(odriveCommand),
          _sensorStates(sensorStates),
          _errorStates(errorStates),
          _motors(MOTOR_SUBSCRIBER_NAME, &rosHelper::receiveJointState, this),
          _odriveCmd(ODRIVE_SUBSCRIBER_NAME, &rosHelper::receiveODriveCommand, this),
          _sensors(ENCODER_PUBLISHER_NAME, &_sensorStates),
          _odriveErrors(ODRIVE_ERROR_PUBLISHER_NAME, &_errorStates)
        {}

    void rosHelper::begin() {
        _nh.initNode();
        _nh.subscribe(_motors);
        _nh.subscribe(_odriveCmd);
        _nh.advertise(_sensors);
        _nh.advertise(_odriveErrors);
    }

    void rosHelper::receiveJointState(const sensor_msgs::JointState &msg) {

    }

    void rosHelper::receiveODriveCommand(const sensor_msgs::Joy &msg) {

    }

    void rosHelper::publishSensorStates() {

    }

    void rosHelper::publishErrorState() {
        _odriveErrors.publish(&_errorStates);
    }

    int rosHelper::spinOnce() {
        _nh.spinOnce();
    }
