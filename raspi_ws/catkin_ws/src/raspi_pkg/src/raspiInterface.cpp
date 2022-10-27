#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "teensySubscriber.cpp"
#include <cstdio>

//RaspiInterface allows one to control the rimeless wheel with joystick or trained controller.
//It acts as a bridge between control command generator and teensy

int main(int argc, char **argv){

    TeensySubscriberCallBack teensyCb;
    ros::init(argc, argv, "raspi_pkg_node");
    printf("Starting node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("sensors", 1000, &TeensySubscriberCallBack::getSensorStates, &teensyCb);
    
    //TODO: receive torque through ROS bridge
    //First publish any joint state in your PC's terminal "rostopic pub -r 10 /torque sensor_msgs/JointState  '{effort:  {0.0}'"
    //On raspi side, subscribe to /torque " rostopic sub /torque ...."
    //Then add ros bridge  subscribing to this file.
    //TO launch, call "roslaunch raspi_ws raspi.launch"
    ros::spin();

    return 0;
}