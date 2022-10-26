#include "ros/ros.h"
#include "teesySubscriber.cpp"

int main(int argc, char* argv[]){

    teensyCb = TeensySubscriberCallBack();
    ros::init(argc, argv, "raspi");
    ros::NodeHandle nh;
    ros::Subscriber sub = n.subscribe("/sensors", 1000, &teensyCb.getSensorStates);
    
    //TODO: receive torque through ROS bridge
    //First publish any joint state in your PC's terminal "rostopic pub -r 10 /torque sensor_msgs/JointState  '{effort:  {0.0}'"
    //On raspi side, subscribe to /torque " rostopic sub /torque ...."
    //THen add ros bridge  subscribing to this file.
    //TO launch, call "roslaunch raspi_ws raspi.launch"
    
    ros::spin();

    return 0;
}