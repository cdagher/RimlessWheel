
#include "ros/ros.h"
#include "teesySubscriber.cpp"

int main(int argc, char* argv[]){

    teensyCb = TeensySubscriberCallBack();
    ros::init(argc, argv, "raspi");
    ros::NodeHandle nh;
    ros::Subscriber sub = n.subscribe("/sensors", 1000, teensyCb);
    //TODO: receive torque through ROS bridge
    
    ros::spin();

    return 0;
}