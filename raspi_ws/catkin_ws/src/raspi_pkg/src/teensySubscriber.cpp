
class TeensySubscriberCallBack{

    public: 
        float torsoState[2];
        float spokeState[4];

        void getSensorStates(const sensor_msgs::JointState &msg) {

            torsoState[0] = msg.position[0];
            torsoState[1] = msg.velocity[0];
            spokeState[0] = msg.position[1];
            spokeState[1] = msg.position[2];
            spokeState[2] = msg.velocity[1];
            spokeState[3] = msg.velocity[2];

            ROS_INFO("Torso angle = %f", torsoState[0]);    

        }   

        void isTurning(){
            
        } 

};