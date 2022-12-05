#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "teensySubscriber.cpp"
#include <cstdio>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

//RaspiInterface allows one to control the rimeless wheel with joystick or trained controller.
//It acts as a bridge between control command generator and teensy

void joystickCb(const boost::shared_ptr<geometry_msgs::Twist const> msg, ros::Publisher& pub){
    //populate the content of joystickCommand (AKA joystickCommand.effort) from what you 
    //receive through ros bridge
    sensor_msgs::JointState joystickCommand;
    joystickCommand.velocity.resize(1);
    joystickCommand.velocity[0] = {msg->angular.z};
    pub.publish(joystickCommand);
}

int main(int argc, char **argv){

    // TeensySubscriberCallBack teensyCb;
    ros::init(argc, argv, "raspi_pkg_node");
    printf("Starting node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/torso_command", 1);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, boost::bind(&joystickCb, _1, boost::ref(pub)));

    while (ros::ok()){
        ros::spinOnce();
    }
    //TODO: receive torque through ROS bridge
    //First publish any joint state in your PC's terminal "rostopic pub -r 10 /torque sensor_msgs/JointState  '{effort:  {0.0}'"
    //On raspi side, subscribe to /torque " rostopic sub /torque ...."
    //Then add ros bridge  subscribing to this file.
    //TO launch, call "roslaunch raspi_ws raspi.launch"

    return 0;
}