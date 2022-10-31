#include "walking_lib.h"

void init_high_cmd(){
    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.mode = IDLE_M;
    cmd.gaitType = IDLE_G;
    cmd.speedLevel = LOW_S;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.position[0] = 0;
    cmd.position[1] = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;
}

void set_move(float vel_x, float vel_y, float yaw_speed){
    cmd.velocity[0] = vel_x;
    cmd.velocity[1] = vel_y;
    cmd.yawSpeed = yaw_speed;
}

void set_pos(float x, float y){
    cmd.position[0] = x;
    cmd.position[1] = y;
}

void set_height(float body_delta, float foot_delta){
    cmd.bodyHeight = body_delta;
    cmd.footRaiseHeight = foot_delta;
}

void set_pitch(float pitch){
    cmd.euler[0] = pitch;
}

void set_body_orientation(float pitch, float roll, float yaw){
    cmd.euler[0] = pitch;
    cmd.euler[1] = roll;
    cmd.euler[2] = yaw;
}

void set_gait(Gait gait){
    cmd.gaitType = gait;
}

void set_speed(Speed speed){
    cmd.speedLevel = speed;
}

void set_mode(Mode mode){
    cmd.mode = mode;
}

void send_cmd(ros::Publisher& pub){
    pub.publish(cmd);
}
