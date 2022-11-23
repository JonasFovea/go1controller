#!/usr/bin/env python3

import rospy
from go1_legged_msgs.msg import JointCmd
from go1_legged_msgs.msg import LowState
from go1_legged_msgs.msg import MotorState

# HIGHLEVEL = 0xEE
# LOWLEVEL = 0xFF
# TRIGERLEVEL = 0xF0
#
# PosStopF = 2.146E+9
# VelStopF = 16000.0
#
# PMSM = 0x0A
# BRAKE = 0x00
#
# target_pose = [-0.28, 1.12, -2.70,
#                0.28, 1.12, -2.70,
#                -0.28, 1.12, -2.70,
#                -0.28, 1.12, -2.70]

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

leg_joints = [[FR_0, FR_1, FR_2], [FL_0, FL_1, FL_2], [RR_0, RR_1, RR_2], [RL_0, RL_1, RL_2]]


class JointController:
    def __init__(self):
        self.joint_positions = [0.0 for _ in range(12)]
        self.joint_velocities = [0.0 for _ in range(12)]
        self.joint_torques = [0.0 for _ in range(12)]
        self.joint_kps = [5.0 for _ in range(12)]
        self.joint_kds = [1.0 for _ in range(12)]

        self.state = LowState()

    def initialize_values_from_state(self, state: LowState):
        for i in range(12):
            self.joint_positions[i] = state.motorState[i].q
            self.joint_velocities[i] = state.motorState[i].dq
            self.joint_torques[i] = state.motorState[i].tau

    def set_position(self, joint_nr: int, pos: float):
        self.joint_positions[joint_nr] = pos

    def get_command(self):
        msg = JointCmd()
        msg.q = self.joint_positions
        msg.dq = self.joint_velocities
        msg.tau = self.joint_torques
        msg.Kp = self.joint_kps
        msg.Kd = self.joint_kds
        return msg

    def get_joint_data_point(self, timestep: int, joint_nr: int):
        return {"t": timestep, "q_is": self.state.motorState[joint_nr].q, "q_set": self.joint_positions[joint_nr]}

    def low_state_callback(self, state: LowState):
        self.state = state


def test():
    joint_controller = JointController()

    pub = rospy.Publisher("/base_node/joint_cmd", JointCmd, queue_size=1)
    rospy.Subscriber("/base_node/state", LowState, joint_controller.low_state_callback, queue_size=10)
    rospy.init_node("low_controller", anonymous=True)
    rate = rospy.Rate(500)

    counter = 0
    dataset = []

    init_pos = -2.0
    end_pos = -1.5

    q0 = 0.0
    q1 = 1.2
    q2 = -2.0

    for leg in leg_joints:
        for joint, q in zip(leg, [q0, q1, q2]):
            joint_controller.set_position(joint, q)

    if not rospy.is_shutdown():
        pub.publish(joint_controller.get_command())
        rate.sleep()

    while not rospy.is_shutdown():
        dataset.append(joint_controller.get_joint_data_point(counter, FR_2))
        counter += 1

        if counter <= 500:
            joint_controller.set_position(FR_2, init_pos)
        elif counter < 1000:
            joint_controller.set_position(FR_2, end_pos)
        else:
            break

        pub.publish(joint_controller.get_command())

        rate.sleep()

    print(f"Collected dataset: \n{dataset}\n\n")


if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException as e:
        print(f"ROS Error: {e}\n\nExiting...")
