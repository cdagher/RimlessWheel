#include <sensor_msgs/JointState.h>


class TeensySubscriberCallBack{

    public: 
        float torsoState[2];
        float wheelState[2];
    
    private: 

        void getSensorStates(sensorStates) {

            torsoState[1] = sensorStates.position[1]
            torsoState[2] = sensorStates.position[2]

            //TODO: get orientation from IMU directly.
            // float sensorStateVals[8] = 
            //     {
            //     encPos0, encPos1, // encoder values
            //     accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, // accelerometer values
            //     gyro.gyro.x, gyro.gyro.y, gyro.gyro.z // gyroscope values
            //     };

        }    

};