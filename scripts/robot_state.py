#!/usr/bin/env python3
import os

import rospy
from unitree_legged_msgs.msg import HighState


def high_state_listen(state: HighState):
    out = "====\tRobot status\t====\n"

    out += f"Control level:\t{'HIGH LEVEL' if state.levelFlag == 0xEE else 'LOW LEVEL'}\n"
    out += f"Motor states:\n"
    for i in range(12):
        out += f"\tMotor {i+1} position {state.motorState[i].q}\n"
    out += "Foot states:\n"
    for i, n in enumerate(["FR", "FL", "RR", "RL"]):
        out += f"\t{n} Foot force: {state.footForce[i]}\n"
    out += f"Gait type: {state.gaitType}\n"
    out += f"Position (xyz): {state.position}\n"
    out += f"Body height: {state.bodyHeight}\n"
    out += f"Velocity (xyz): {state.velocity}\n"
    out += f"Yaw speed: {state.yawSpeed}\n"
    out += f"Acceleration (xyz): {state.imu.accelerometer}\n"
    out += f"Gyroscope (xyz): {state.imu.gyroscope}\n"
    out += f"Orientation (rpy): {state.imu.rpy}\n"

    out += "====\t============\t===="
    os.system("clear")
    print(out)


if __name__ == '__main__':
    rospy.init_node('robot_state', anonymous=False)
    rospy.Subscriber("/high_state", HighState, high_state_listen)
    rospy.spin()
