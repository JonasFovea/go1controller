#ifndef JOYSTICK_CONTROL_H
#define JOYSTICK_CONTROL_H

/**
 * ==========================================================
 * Struct definitions
 * ==========================================================
 */

typedef struct controller{
    int LR, FB, YAW, PITCH;
    int RT, LT;
    int UPDOWN, LEFTRIGHT;

    int A, B, X, Y;
    int UP, DOWN, LEFT, RIGHT;
    int START, SELECT, MENU;
    int RB, LB;
    int RS, LS;

    float resolution;
    int invert;
} controller;

typedef struct robot_state{
    int mode;
    int num_modes;

    int standing;

    float body_height;
} robot_state;

typedef struct button_states{
    int A_S, B_S, X_S, Y_S;
    int UP_S, DOWN_S, LEFT_S, RIGHT_S;
    int START_S, SELECT_S, MENU_S;
    int RB_S, LB_S;
    int RS_S, LS_S;

    int A_T, B_T, X_T, Y_T;
    int UP_T, DOWN_T, LEFT_T, RIGHT_T;
    int START_T, SELECT_T, MENU_T;
    int RB_T, LB_T;
    int RS_T, LS_T;
} button_states;

/**
 * ==========================================================
 * Global variables
 * ==========================================================
 */

unitree_legged_msgs::HighCmd cmd;

controller gamepad;
button_states buttons;
robot_state robot;

const int num_button_groups = 11;
int* button_group_addrs[num_button_groups*3];

ros::Subscriber sub;
ros::Publisher pub;

/**
 * ==========================================================
 * Conversion constants
 * ==========================================================
 */

const float linear_speed_unit = 1.0f; // m/s
const float rotational_speed_unit = 2.0f; //rad/s
const float angle_unit = 0.52;// rad (~30 deg)
const float max_body_height_delta = 0.04f; // m
const float min_body_height_delta = -0.28f; // m
const float height_step = 0.02f; // m

/**
 * ==========================================================
 * Function declarations
 * ==========================================================
 */

void link_button_pairs();
void load_layout(int profile, controller* gamepad);
void update_buttons(const sensor_msgs::Joy::ConstPtr &msg);
void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
void init_states();
void init_high_command();

#endif //JOYSTICK_CONTROL_H
