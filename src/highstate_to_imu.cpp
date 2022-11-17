#include "../include/highstate_to_imu.h"

void state_callback(const go1_legged_msgs::HighState& state){
    msg.header.frame_id = "go1";

    msg.orientation.x = state.imu.quaternion[0];
    msg.orientation.y = state.imu.quaternion[1];
    msg.orientation.z = state.imu.quaternion[2];
    msg.orientation.w = state.imu.quaternion[3];

    msg.angular_velocity.x = state.imu.gyroscope[0];
    msg.angular_velocity.y = state.imu.gyroscope[1];
    msg.angular_velocity.z = state.imu.gyroscope[2];

    msg.linear_acceleration.x = state.imu.accelerometer[0];
    msg.linear_acceleration.y = state.imu.accelerometer[1];
    msg.linear_acceleration.z = state.imu.accelerometer[2];

    pub.publish(msg);
}

int main(int argc, char** argv){
    printf("\n\n\n[i] started highstate_to_imu\n");
    ros::init(argc, argv, "high2imu");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);
    sub = nh.subscribe("/high_state",1, state_callback);
    ros::spin();
    printf("[i] ended highstate_to_imu\n");
}