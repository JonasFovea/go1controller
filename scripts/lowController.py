#!/usr/bin/env python3
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


def init_low_command() -> LowCmd:
    cmd_msg = LowCmd()
    cmd_msg.head = [0xFE, 0xEF]
    cmd_msg.levelFlag = LOWLEVEL

    for mcmd, pos in zip(cmd_msg.motorCmd, target_pose):
        mcmd.mode = PMSM
        mcmd.q = pos
        mcmd.dq = 0
        mcmd.tau = 0
        mcmd.Kp = 1.0
        mcmd.Kd = 5.0

    return cmd_msg


def set_motor_pos_step() -> MotorCmd:
    mcmd = MotorCmd()
    mcmd.mode = PMSM
    mcmd.q = PosStopF
    mcmd.dq = 0
    mcmd.tau = 0
    mcmd.Kp = 50.0
    mcmd.Kd = 5.0
    return mcmd


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

    rate = rospy.Rate(10)

    cmd_msg = init_low_command()

    if not rospy.is_shutdown():
        rospy.loginfo(cmd_msg)
        pub.publish(cmd_msg)
        rate.sleep()

        counter += 1
        cmd_msg.motorCmd[2] = set_motor_pos_step()
        rospy.loginfo(cmd_msg)
        pub.publish(cmd_msg)
        rate.sleep()

    while not rospy.is_shutdown():
        if counter > 10000:
            break

        counter += 1
        rate.sleep()

    print("Collected 10000 samples:")
    for s in samples:
        print(f"\tt step: {s[0]:5d}\tq: {s[1]}\t q_raw: {s[2]}")




if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException as e:
        print(f"ROS Error: {e}\n\nExiting...")
