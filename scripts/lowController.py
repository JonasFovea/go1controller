#!/usr/bin/env python3
import math

import rospy
from go1_legged_msgs.msg import LowCmd
from go1_legged_msgs.msg import LowState
from go1_legged_msgs.msg import MotorCmd
from go1_legged_msgs.msg import MotorState

HIGHLEVEL = 0xEE
LOWLEVEL = 0xFF
TRIGERLEVEL = 0xF0

PosStopF = 2.146E+9
VelStopF = 16000.0

PMSM = 0x0A
BRAKE = 0x00

target_pose = [-0.28, 1.12, -2.70,
               0.28, 1.12, -2.70,
               -0.28, 1.12, -2.70,
               -0.28, 1.12, -2.70]

counter = 0
samples = []

FR_0 = 0
FR_1 = 1
FR_2 = 2

FL_0 = 3
FL_1 = 4
FL_2 = 5

RR_0 = 6
RR_1 = 7
RR_2 = 8

RL_0 = 9
RL_1 = 10
RL_2 = 11


def init_low_command() -> LowCmd:
    cmd_msg = LowCmd()
    cmd_msg.head = [0xFE, 0xEF]
    cmd_msg.levelFlag = LOWLEVEL

    for mcmd in cmd_msg.motorCmd:
        mcmd.mode = PMSM
        mcmd.q = PosStopF
        mcmd.dq = VelStopF
        mcmd.tau = 0
        mcmd.Kp = 0
        mcmd.Kd = 0
    print(f"Initialized low command:\n{cmd_msg}")
    return cmd_msg


def set_joint_positions(motiontime) -> LowCmd:
    cmd_msg = init_low_command()

    cmd_msg.motorCmd[FR_0].tau = -0.65
    cmd_msg.motorCmd[FL_0].tau = +0.65
    cmd_msg.motorCmd[RR_0].tau = -0.65
    cmd_msg.motorCmd[RL_0].tau = +0.65

    cmd_msg.motorCmd[FR_2].q = -math.pi/2 + 0.5 * math.sin(2 * math.pi / 5.0 * motiontime * 1e-3)
    cmd_msg.motorCmd[FR_2].dq = 0.0
    cmd_msg.motorCmd[FR_2].Kp = 5.0
    cmd_msg.motorCmd[FR_2].Kd = 1.0

    cmd_msg.motorCmd[FR_0].q = 0.0
    cmd_msg.motorCmd[FR_0].dq = 0.0
    cmd_msg.motorCmd[FR_0].Kp = 5.0
    cmd_msg.motorCmd[FR_0].Kd = 1.0

    cmd_msg.motorCmd[FR_1].q = 0.0
    cmd_msg.motorCmd[FR_1].dq = 0.0
    cmd_msg.motorCmd[FR_1].Kp = 5.0
    cmd_msg.motorCmd[FR_1].Kd = 1.0

    return cmd_msg


def low_state_tracker(msg: LowState):
    global counter  # yes, I didn't want to do this, but it's the fastest solution
    # global samples  # yes, I didn't want to do this, but it's the fastest solution
    print(f"Sample recorded for {counter=:5d}")
    samples.append((counter, msg.motorState[2].q, msg.motorState[2].q_raw))



def test():
    pub = rospy.Publisher("go1_controller/low_cmd", LowCmd, queue_size=1)
    rospy.Subscriber("go1_controller/low_state", LowState, low_state_tracker, queue_size=10)
    rospy.init_node("low_controller", anonymous=True)

    global counter  # yes, I didn't want to do this, but it's the fastest solution

    rate = rospy.Rate(500)

    cmd_msg = init_low_command()

    motiontime = 0

    while not rospy.is_shutdown():
        if counter > 10:
            cmd_msg = set_joint_positions(motiontime)
            motiontime += 2
        if counter > 10000:
            break

        counter += 1
        pub.publish(cmd_msg)
        rate.sleep()

    print("Collected 10000 samples:")
    for s in samples:
        print(f"\tt step: {s[0]:5d}\tq: {s[1]}\t q_raw: {s[2]}")




if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException as e:
        print(f"ROS Error: {e}\n\nExiting...")
