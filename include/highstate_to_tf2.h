#ifndef HIGHSTATE_TO_TF2_H
#define HIGHSTATE_TO_TF2_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

#include <go1_legged_msgs/HighState.h>

ros::Subscriber sub;
tf2_ros::TransformBroadcaster* tfb;
geometry_msgs::TransformStamped go1_transform;
geometry_msgs::TransformStamped foot_transform;
geometry_msgs::TransformStamped sensor_transform;

const char* feet[] = {"FR", "FL", "RR", "RL"};

void state_callback(const go1_legged_msgs::HighState& state);

#endif  // HIGHSTATE_TO_TF2_H
