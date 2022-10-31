#ifndef WALKING_LIB_H
#define WALKING_LIB_H

#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


typedef enum gait {IDLE_G, TROT_G, RUN_G, STAIR_G, OBSTACLE_G} Gait;
typedef enum speed {LOW_S, MEDIUM_S, HIGH_S} Speed;
typedef enum mode {IDLE_M, STAND_M, WALK_VEL_M, WALK_POS_M,
                   WALK_PATH_M, DOWN_M, UP_M, DAMP_M, RECOVER_M,
                   BACKFLIP_M, JUMP_YAW_M, GREET_M, DANCE1_M, DANCE2_M} Mode;


unitree_legged_msgs::HighCmd cmd;

void init_high_cmd();

void set_move(float vel_x, float vel_y, float yaw_speed);
void set_pos(float x, float y);

void set_height(float body_delta, float foot_delta);

void set_pitch(float pitch);
void set_body_orientation(float pitch, float roll, float yaw);

void set_gait(Gait gait);
void set_speed(Speed speed);
void set_mode(Mode mode);

void send_cmd(ros::Publisher& pub);

#endif //WALKING_LIB_H
