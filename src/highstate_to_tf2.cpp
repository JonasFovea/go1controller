#include "../include/highstate_to_tf2.h"


void state_callback(const unitree_legged_msgs::HighState& state){
    go1_transform.header.frame_id = "world";
    go1_transform.child_frame_id = "go1";

    go1_transform.transform.translation.x = state.position[0];
    go1_transform.transform.translation.y = state.position[1];
    go1_transform.transform.translation.z = state.position[2];

    tf2::Quaternion q;
    q.setRPY(state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]);
    go1_transform.transform.rotation.x = q.x();
    go1_transform.transform.rotation.y = q.y();
    go1_transform.transform.rotation.z = q.z();
    go1_transform.transform.rotation.w = q.w();
    
    go1_transform.header.stamp = ros::Time::now();
    tfb->sendTransform(go1_transform);

    for (int i=0; i<4; ++i){
        foot_transform.header.frame_id = "go1";
        foot_transform.child_frame_id = feet[i];

        foot_transform.transform.translation.x = state.footPosition2Body[i].x;
        foot_transform.transform.translation.y = state.footPosition2Body[i].y;
        foot_transform.transform.translation.z = state.footPosition2Body[i].z;

        tf2::Quaternion q0;
        q0.setRPY(0, 0, 0);
        foot_transform.transform.rotation.x = q0.x();
        foot_transform.transform.rotation.y = q0.y();
        foot_transform.transform.rotation.z = q0.z();
        foot_transform.transform.rotation.w = q0.w();

        foot_transform.header.stamp = ros::Time::now();
        tfb->sendTransform(foot_transform);
    }

    sensor_transform.header.frame_id = "go1";
    sensor_transform.child_frame_id = "go1_front";
    sensor_transform.transform.translation.x = 0.30;
    sensor_transform.transform.translation.y = 0.0;
    sensor_transform.transform.translation.z = 0.0;
    tf2::Quaternion q1;
    q1.setRPY(0, 0, 0);
    sensor_transform.transform.rotation.x = q1.x();
    sensor_transform.transform.rotation.y = q1.y();
    sensor_transform.transform.rotation.z = q1.z();
    sensor_transform.transform.rotation.w = q1.w();
    sensor_transform.header.stamp = ros::Time::now();
    tfb->sendTransform(sensor_transform);

    tf2::Quaternion q2;
    sensor_transform.header.frame_id = "go1";
    sensor_transform.child_frame_id = "go1_left";
    sensor_transform.transform.translation.x = -0.06;
    sensor_transform.transform.translation.y = 0.10;
    sensor_transform.transform.translation.z = 0.0;
    q2.setRPY(0, 0, 1.5708f);
    sensor_transform.transform.rotation.x = q2.x();
    sensor_transform.transform.rotation.y = q2.y();
    sensor_transform.transform.rotation.z = q2.z();
    sensor_transform.transform.rotation.w = q2.w();
    sensor_transform.header.stamp = ros::Time::now();
    tfb->sendTransform(sensor_transform);

    tf2::Quaternion q3;
    sensor_transform.header.frame_id = "go1";
    sensor_transform.child_frame_id = "go1_right";
    sensor_transform.transform.translation.x = -0.06;
    sensor_transform.transform.translation.y = -0.10;
    sensor_transform.transform.translation.z = 0.0;
    q3.setRPY(0, 0, -1.5708f);
    sensor_transform.transform.rotation.x = q3.x();
    sensor_transform.transform.rotation.y = q3.y();
    sensor_transform.transform.rotation.z = q3.z();
    sensor_transform.transform.rotation.w = q3.w();
    sensor_transform.header.stamp = ros::Time::now();
    tfb->sendTransform(sensor_transform);

}

int main(int argc, char **argv){
    printf("\n\n\n[i] started highstate_to_tf2\n");
    ros::init(argc, argv, "high2tf2");
    ros::NodeHandle nh;

    tfb = new tf2_ros::TransformBroadcaster();
    sub = nh.subscribe("/high_state",1, state_callback);
    ros::spin();
    printf("[i] ended highstate_to_tf2\n");
}
