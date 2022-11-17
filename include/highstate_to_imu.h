#ifndef HIGHSTATE_TO_IMU_H
#define HIGHSTATE_TO_IMU_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

#include <go1_legged_msgs/HighState.h>

ros::Subscriber sub;
ros::Publisher pub;
sensor_msgs::Imu msg;

void state_callback(const go1_legged_msgs::HighState& state);

#endif //HIGHSTATE_TO_IMU_H
