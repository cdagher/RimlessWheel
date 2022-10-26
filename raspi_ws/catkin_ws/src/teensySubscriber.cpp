#include <sensor_msgs/JointState.h>


class TeensySubscriberCallBack{

    public: 
        float torsoState[2];
        float spokeState[4];

        void getSensorStates(const sensor_msgs::JointState &sensorStates) {

            torsoState[1] = sensorStates.position[1];
            torsoState[2] = sensorStates.velocity[1];
            spokeState[1] = sensorStates.position[2];
            spokeState[2] = sensorStates.position[3];
            spokeState[3] = sensorStates.velocity[2];
            spokeState[4] = sensorStates.velocity[3];

        }   

        void isTurning(){
            
        } 

};