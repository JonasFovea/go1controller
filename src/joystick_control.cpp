#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <sensor_msgs/Joy.h>

using namespace UNITREE_LEGGED_SDK;

typedef struct controller{
    int LR, FB, YAW, PITCH;
    int RT, LT;
    int UPDOWN, LEFTRIGHT;

    int A, B, X, Y;
    int UP, DOWN, LEFT, RIGHT;
    int START, SELECT, MENU;
    int RB, LB;
    int RS, LS;

    int resolution;
    int invert;
} controller;

typedef struct states{
    int A_S, B_S, X_S, Y_S;
    int UP_S, DOWN_S, LEFT_S, RIGHT_S;
    int START_S, SELECT_S, MENU_S;
    int RB_S, LB_S;
    int RS_S, LS_S;

    int mode;
    int num_modes;
} states;

unitree_legged_msgs::HighCmd cmd;

controller gamepad;
states robot_state;

ros::Subscriber sub;
ros::Publisher pub;

float linear_speed_unit = 1.0f; // m/s
float rotational_speed_unit = 2.0f; //rad/s
float angle_unit = 0.17;// rad (~10 deg)

void load_layout(int profile, controller* gamepad){
    switch (profile) {
        default: //Logitech Gamepad F310
            // Axis
            gamepad->FB = 1;
            gamepad->LR = 0;
            gamepad->YAW = 3;
            gamepad->PITCH = 4;

            gamepad->UPDOWN = 7;
            gamepad->LEFTRIGHT = 6;

            gamepad->RT = 5;
            gamepad->LT = 2;

            // Buttons
            gamepad->A = 0;
            gamepad->B = 1;
            gamepad->X = 2;
            gamepad->Y = 3;

            gamepad->START = 7;
            gamepad->SELECT = 6;
            gamepad->MENU = 8;

            gamepad->RB = 5;
            gamepad->LB = 4;

            gamepad->RS = 10;
            gamepad->LS = 9;

            // Settings
            gamepad->invert = 0;
            gamepad->resolution = 32767;

            gamepad->UP = -1;
            gamepad->DOWN = -1;
            gamepad->LEFT = -1;
            gamepad->RIGHT = -1;
    }
}

void joy_callback(const sensor_msgs::Joy::ConstPtr &msg){
    cmd.velocity[0] = msg->axes[gamepad.FB]/gamepad.resolution * linear_speed_unit;
    cmd.velocity[1] = msg->axes[gamepad.LR]/gamepad.resolution * linear_speed_unit;

    cmd.yawSpeed = msg->axes[gamepad.YAW]/gamepad.resolution * rotational_speed_unit;
    cmd.euler[1] = msg->axes[gamepad.PITCH]/gamepad.resolution * angle_unit;

    // Toggle next mode via START button
    if (msg->axes[gamepad.START]){
        if(!robot_state.START_S){
            robot_state.mode = (robot_state.mode + 1) % robot_state.num_modes;
            robot_state.START_S = 1;
        }
    } else {
        robot_state.START_S = 0;
    }

    cmd.mode = robot_state.mode;
    
    pub.publish(cmd);
}

void init_states(){
    robot_state = {0, 0,0,0,
               0,0,0,0,
               0,0,0,
               0,0,0,0,
               0, 3};
}

void init_high_command(){
    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.levelFlag = HIGHLEVEL;
    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "joystick_control");

    ros::NodeHandle nh;

    init_high_command();
    init_buttons();

    sub = nh.subscribe("joy",1, joy_callback);
    pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

    while (ros::ok());
    return 0;
}