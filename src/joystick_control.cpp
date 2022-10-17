#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>

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

unitree_legged_msgs::HighCmd cmd;

controller gamepad;
button_states buttons;
robot_state robot;

ros::Subscriber sub;
ros::Publisher pub;

float linear_speed_unit = 1.0f; // m/s
float rotational_speed_unit = 2.0f; //rad/s
float angle_unit = 0.52;// rad (~30 deg)
float max_body_height_delta = 0.06f; // m
float min_body_height_delta = -0.28f; // m

void load_layout(int profile, controller* gamepad){

    // disable D-pad Axes/Buttons
    gamepad->UP = -1;
    gamepad->DOWN = -1;
    gamepad->LEFT = -1;
    gamepad->RIGHT = -1;
    gamepad->UPDOWN = -1;
    gamepad->LEFTRIGHT = -1;


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
            gamepad->resolution = 1.0f;
            break;
    }
}

void  update_buttons(const sensor_msgs::Joy::ConstPtr &msg){
    printf("[i] button update start\n");
    // Button A
    if (msg->buttons[gamepad.A]){
        buttons.A_T = buttons.A_S ? 0 : 1;
        buttons.A_S = 1;
    }else{
        buttons.A_T = 0;
        buttons.A_S = 0;
    }

    // Button B
    if (msg->buttons[gamepad.B]){
        buttons.B_T = buttons.B_S ? 0 : 1;
        buttons.B_S = 1;
    }else{
        buttons.B_T = 0;
        buttons.B_S = 0;
    }

    // Button X
    if (msg->buttons[gamepad.X]){
        buttons.X_T = buttons.X_S ? 0 : 1;
        buttons.X_S = 1;
    }else{
        buttons.X_T = 0;
        buttons.X_S = 0;
    }

    // Button Y
    if (msg->buttons[gamepad.Y]){
        buttons.Y_T = buttons.Y_S ? 0 : 1;
        buttons.Y_S = 1;
    }else{
        buttons.Y_T = 0;
        buttons.Y_S = 0;
    }

    if (msg->buttons[gamepad.RB]){
        buttons.RB_T =  buttons.RB_S ? 0 : 1;
        buttons.RB_S = 1;
    }else{
        buttons.RB_T = 0;
        buttons.RB_S = 0;
    }

    if (msg->buttons[gamepad.LB]){
        buttons.LB_T = buttons.LB_S ? 0 : 1;
        buttons.LB_S = 1;
    }else{
        buttons.LB_T = 0;
        buttons.A_S = 0;
    }

    if (msg->buttons[gamepad.RS]){
        buttons.RS_T = buttons.RS_S ? 0 : 1;
        buttons.RS_S = 1;
    }else{
        buttons.RS_T = 0;
        buttons.RS_S = 0;
    }

    if (msg->buttons[gamepad.LS]){
        buttons.LS_T = buttons.LS_S ? 0 : 1;
        buttons.LS_S = 1;
    }else{
        buttons.LS_T = 0;
        buttons.LS_S = 0;
    }

    if (msg->buttons[gamepad.START]){
        buttons.START_T = buttons.START_S ? 0 : 1;
        buttons.START_S = 1;
    }else{
        buttons.START_T = 0;
        buttons.START_S = 0;
    }

    if (msg->buttons[gamepad.SELECT]){
        buttons.SELECT_T = buttons.SELECT_S ? 0 : 1;
        buttons.SELECT_S = 1;
    }else{
        buttons.SELECT_T = 0;
        buttons.SELECT_S = 0;
    }

    if (msg->buttons[gamepad.MENU]){
        buttons.MENU_T = buttons.MENU_S ? 0 : 1;
        buttons.MENU_S = 1;
    }else{
        buttons.MENU_T = 0;
        buttons.MENU_S = 0;
    }
    
    if (gamepad.LEFTRIGHT == -1) {
        if (gamepad.LEFT != -1) {
            if (msg->buttons[gamepad.LEFT]){
                buttons.LEFT_T = buttons.LEFT_S ? 0 : 1;
                buttons.LEFT_S = 1;
            }else{
                buttons.LEFT_T = 0;
                buttons.LEFT_S = 0;
            }
        }
        if (gamepad.RIGHT != -1) {
            if (msg->buttons[gamepad.RIGHT]){
                buttons.RIGHT_T = buttons.RIGHT_S ? 0 : 1;
                buttons.RIGHT_S = 1;
            }else{
                buttons.RIGHT_T = 0;
                buttons.RIGHT_S = 0;
            }
        }
    }
    else{
        float ax_val = (float) msg->axes[gamepad.LEFTRIGHT];
        if (ax_val != 0.0){
            buttons.LEFT_T = buttons.LEFT_S ? 0 : 1;
            buttons.LEFT_S = ax_val > 0.0f ? 1 : 0;
            buttons.RIGHT_T = buttons.RIGHT_S ? 0 : 1;
            buttons.RIGHT_S = ax_val < 0.0f ? 1 : 0;
        }else{
            buttons.LEFT_S = 0;
            buttons.LEFT_T = 0;
            buttons.RIGHT_S = 0;
            buttons.RIGHT_T = 0;
        }
    }

    if (gamepad.UPDOWN == -1) {
        if (gamepad.UP != -1) {
            if (msg->buttons[gamepad.UP]){
                buttons.UP_T = buttons.UP_S ? 0 : 1;
                buttons.UP_S = 1;
            }else{
                buttons.UP_T = 0;
                buttons.UP_S = 0;
            }
        }
        if (gamepad.DOWN != -1) {
            if (msg->buttons[gamepad.DOWN]){
                buttons.DOWN_T = buttons.DOWN_S ? 0 : 1;
                buttons.DOWN_S = 1;
            }else{
                buttons.DOWN_T = 0;
                buttons.DOWN_S = 0;
            }
        }
    }
    else{
        float ax_val = (float) msg->axes[gamepad.UPDOWN];
        if (ax_val != 0.0){
            buttons.UP_T = buttons.UP_S ? 0 : 1;
            buttons.UP_S = ax_val > 0.0f ? 1 : 0;
            buttons.DOWN_T = buttons.DOWN_S  ? 0 : 1;
            buttons.DOWN_S = ax_val < 0.0f ? 1 : 0;
        }else{
            buttons.UP_T = 0;
            buttons.UP_S = 0;
            buttons.DOWN_T = 0;
            buttons.DOWN_S = 0;
        }
    }
    printf("[i] button update done.\n");
}

void joy_callback(const sensor_msgs::Joy::ConstPtr &msg){
    printf("[i] joy_callback\n");

//    printf("%f, %f, %f, %f, %f, %f, %f, %f\n", msg->axes[0],msg->axes[1],msg->axes[2],msg->axes[3],msg->axes[4],msg->axes[5],msg->axes[6],msg->axes[7]);
    printf("FB: %f LR: %f YAW: %f PITCH: %f\n",msg->axes[gamepad.FB], msg->axes[gamepad.LR], msg->axes[gamepad.YAW], msg->axes[gamepad.PITCH]);
    cmd.velocity[0] = ((float) msg->axes[gamepad.FB])/gamepad.resolution * linear_speed_unit;
    cmd.velocity[1] = ((float) msg->axes[gamepad.LR])/gamepad.resolution * linear_speed_unit;

    cmd.yawSpeed =  ((float) msg->axes[gamepad.YAW])/gamepad.resolution * rotational_speed_unit;
    cmd.euler[1] = ((float) msg->axes[gamepad.PITCH])/gamepad.resolution * angle_unit;

    update_buttons(msg);
    printf("[i] checking modes and buttons\n");
    // robot is standing / walking
    if (robot.mode >= 0 && robot.mode <= 2 ){
        // switch
        if (buttons.START_T) {
            printf("\t[i] start pressed\n");
            robot.mode = (robot.mode + 1) % robot.num_modes;
            robot.standing =  1;
            cmd.mode = robot.mode;
            printf("\n[i] switched mode to %i", robot.mode);
        }

        if (buttons.UP_T){
            printf("\t[i] UP pressed\n");
            robot.body_height += 0.02f; // increase by 2cm
            robot.body_height = robot.body_height > max_body_height_delta ? max_body_height_delta : robot.body_height;
            robot.standing = 1;
            cmd.bodyHeight = robot.body_height;
            printf("\t[i] height increased\n");
        }

        if (buttons.DOWN_T){
            printf("\t[i] DOWN pressed\n");
            robot.body_height -= 0.02f; // decrease by 2cm
            robot.body_height = robot.body_height < min_body_height_delta ? min_body_height_delta : robot.body_height;
            robot.standing = 1;
            cmd.bodyHeight = robot.body_height;
            printf("\t[i] height decreased\n");
        }

        if (buttons.LB_S && buttons.A_T){
            printf("\t[i] LB + A pressed\n");
            robot.mode = 5;
            robot.standing = 0;
            cmd.mode = robot.mode;
            printf("\t[i] mode changed to %i\n", robot.mode);
        }
    }
    // robot is standing down
    else if (robot.mode == 5){
        // stand up
        if (buttons.LB_S && buttons.A_T || buttons.START_T){
            robot.mode = 6;
            robot.standing = 1;
            cmd.mode = robot.mode;
        }
        // dampen
        if (buttons.LB_S && buttons.B_T){
            robot.mode = 7;
            cmd.mode = robot.mode;
        }
    }
    // robot is standing up
    else if (robot.mode == 6){
        // stand down
        if (buttons.LB_S && buttons.A_T){
            robot.mode = 5;
            robot.standing = 0;
            cmd.mode = robot.mode;
        }
        //dampen
        if (buttons.LB_S && buttons.B_T){
            robot.mode = 7;
            robot.standing = 1;
            cmd.mode = robot.mode;
        }
        // idle
        if (buttons.START_T){
            robot.mode = 0;
            robot.standing = 1;
            cmd.mode = robot.mode;
        }
    }
    // robot is dampened
    else if (robot.mode == 7){
        if (buttons.START_T){
            // idle
            if (robot.standing){
                robot.mode = 0;
                robot.standing = 1;
                cmd.mode =  robot.mode;
            }
            // stand up
            else {
                robot.mode = 6;
                robot.standing = 1;
                cmd.mode = robot.mode;
            }
        }

        if (buttons.LB_S && buttons.A_T){
            // stand down
            if (robot.standing){
                robot.mode = 5;
                robot.standing = 1;
                cmd.mode =  robot.mode;
            }
            // stand up
            else {
                robot.mode = 6;
                robot.standing = 1;
                cmd.mode = robot.mode;
            }
        }
    }
    printf("[i] publishing HighCmd\n");
    pub.publish(cmd);
    printf("[i] end of callback\n\n");
}

void init_states(){
    buttons = {0, 0,0,0,
               0,0,0,0,
               0,0,0,
               0,0,0, 0,
               0, 0,0,0,
               0,0,0,0,
               0,0,0,
               0,0,0, 0};

    robot = {0,3,1, 0.0f};
}

void init_high_command(){
    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.levelFlag = HIGHLEVEL;
    cmd.mode = 0;
        // 0. idle, default stand
        // 1. force stand (controlled by dBodyHeight + ypr)
        // 2. target velocity walking (controlled by velocity + yawSpeed)
        // 3. target position walking (controlled by position + ypr[0])
        // 4. path mode walking (reserve for future release)
        // 5. position stand down.
        // 6. position stand up
        // 7. damping mode
        // 8. recovery stand
        // 9. backflip
        // 10. jumpYaw
        // 11. straightHand
        // 12. dance1
        // 13. dance2

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
    printf("[i] Started joystick_control\n");
    ros::init(argc, argv, "joystick_control");

    ros::NodeHandle nh;

    load_layout(0, &gamepad);
    init_high_command();
    init_states();

    sub = nh.subscribe("/joy",1, joy_callback);
    pub = nh.advertise<unitree_legged_msgs::HighCmd>("/high_cmd", 1000);
    printf("[i] joystick_control publisher and subscriber established\n");
    ros::spin();
    printf("[i] Stopped joystick_control\n");
    return 0;
}