#!/usr/bin/env python3
import os

import rospy
from unitree_legged_msgs.msg import HighState


def high_state_listen(state: HighState):
    out = "========\tRobot status\t========\n"

    out += f"Control level:\t{'HIGH LEVEL' if state.levelFlag == 0xEE else 'LOW LEVEL'}\n"

    out += f"Motor states:\n"
    for i in range(12):
        out += f"\tMotor {i+1} position {state.motorState[i].q:0.6f}\n"

    out += "Foot states:\n"
    for i, n in enumerate(["FR", "FL", "RR", "RL"]):
        out += f"\t{n} Foot force: {state.footForce[i]}\n"
    out += f"Gait type: {state.gaitType}\n"
    out += f"Position (xyz): ({state.position[0]:0.3f}, {state.position[1]:0.3f}, {state.position[2]:0.3f})\n"
    out += f"Body height: {state.bodyHeight:0.4f}\n"
    out += f"Velocity (xyz): ({state.velocity[0]:0.5f}, {state.velocity[1]:0.5f}, {state.velocity[2]:0.5f})\n"
    out += f"Yaw speed: {state.yawSpeed:0.5f}\n"

    out += f"Acceleration (xyz): ({state.imu.accelerometer[0]:0.5f}, " \
           f"{state.imu.accelerometer[1]:0.5f}, {state.imu.accelerometer[2]:0.5f})\n "
    out += f"Gyroscope (xyz): ({state.imu.gyroscope[0]:0.5f}, " \
           f"{state.imu.gyroscope[1]:0.5f}, {state.imu.gyroscope[2]:0.5f})\n "

    out += f"Orientation (rpy): ({state.imu.rpy[0]:0.5f}, {state.imu.rpy[1]:0.5f}, {state.imu.rpy[2]:0.5f})\n"

    out += "===================================="
    os.system("clear")
    print(out)


if __name__ == '__main__':
    rospy.init_node('robot_state', anonymous=False)
    rospy.Subscriber("/high_state", HighState, high_state_listen)
    rospy.spin()

# Longest current line:
# Gyroscope (xyz): (-0.005505406763404608, -0.013431791216135025, -0.0015405142912641168)
