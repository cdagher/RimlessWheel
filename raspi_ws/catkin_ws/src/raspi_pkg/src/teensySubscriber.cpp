
class TeensySubscriberCallBack{

    public: 
        float torsoState[2];
        float spokeState[4];

        void getSensorStates(const sensor_msgs::JointState &msg) {

            auto positions = msg.position;
            torsoState[1] = msg.position[1];
            torsoState[2] = msg.velocity[1];
            spokeState[1] = msg.position[2];
            spokeState[2] = msg.position[3];
            spokeState[3] = msg.velocity[2];
            spokeState[4] = msg.velocity[3];

        }   

        void isTurning(){
            
        } 

};