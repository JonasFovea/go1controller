#include "../include/highstate_to_tf2.h"


void state_callback(const unitree_legged_msgs::HighState& state){
    go1_transform.header.frame_id = "world";
    go1_transform.child_frame_id = "go1";

    go1_transform.transform.translation.x = state.position[0];
    go1_transform.transform.translation.y = state.position[1];
    go1_transform.transform.translation.z = state.position[2];

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    go1_transform.transform.rotation.x = q.x();
    go1_transform.transform.rotation.y = q.y();
    go1_transform.transform.rotation.z = q.z();
    go1_transform.transform.rotation.w = q.w();
    
    go1_transform.header.stamp = ros::Time::now();
    tfb.sendTransform(go1_transform);

    for (int i=0; i<4;++i){
        foot_transform.header.frame_id = "go1";
        foot_transform.child_frame_id = feet[i];

        foot_transform.transform.translation.x = state.footPosition2Body[i].x;
        foot_transform.transform.translation.y = state.footPosition2Body[i].y;
        foot_transform.transform.translation.z = state.footPosition2Body[i].z;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        foot_transform.transform.rotation.x = q.x();
        foot_transform.transform.rotation.y = q.y();
        foot_transform.transform.rotation.z = q.z();
        foot_transform.transform.rotation.w = q.w();

        foot_transform.header.stamp = ros::Time::now();
        tfb.sendTransform(foot_transform);
    }

}


int main(int argc, char** argv){
    ros::init(argc, argv, "high2tf2");
    ros::NodeHandle nh;

    sub = nh.subscribe("/high_state",1, state_callback);
}
