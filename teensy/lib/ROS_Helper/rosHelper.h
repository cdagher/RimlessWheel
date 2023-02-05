#ifndef rosHelper_h
#define rosHelper_h

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

#include "ROS_Node_Names.h"

class rosHelper {
public:

    rosHelper(
        sensor_msgs::JointState& motorStates,
        sensor_msgs::Joy& odriveCommand,
        sensor_msgs::JointState& sensorStates, 
        std_msgs::Int32MultiArray& errorStates);
    void begin();

    void receiveJointState(const sensor_msgs::JointState &msg);
    void receiveODriveCommand(const sensor_msgs::Joy &msg);
    void publishSensorStates();
    void publishErrorState();

    int spinOnce();

private:

    ros::NodeHandle _nh;

    ros::Subscriber<sensor_msgs::JointState, rosHelper> _motors;
    ros::Subscriber<sensor_msgs::Joy, rosHelper> _odriveCmd;
    ros::Publisher _sensors;
    ros::Publisher _odriveErrors;

    sensor_msgs::JointState& _motorStates; // comands for the motors
    sensor_msgs::Joy& _odriveCommand; // commands for clearing errors, rebooting, etc
    sensor_msgs::JointState& _sensorStates; // feedback of the encoder positions
    std_msgs::Int32MultiArray& _errorStates; // error states for the ODrive

};

#endif