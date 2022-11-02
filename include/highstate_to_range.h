#ifndef HIGHSTATE_TO_RANGE_H
#define HIGHSTATE_TO_RANGE_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <unitree_legged_msgs/HighState.h>

ros::Subscriber sub;
ros::Publisher pubs[3];
sensor_msgs::Range msg;

const char * sensors[] = {"front", "left", "right"};

void state_callback(const unitree_legged_msgs::HighState& state);

#endif //HIGHSTATE_TO_RANGE_H
