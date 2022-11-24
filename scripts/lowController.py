#!/usr/bin/env python3
import numpy as np
import rospy
import matplotlib.pyplot as plt

from go1_legged_msgs.msg import JointCmd
from go1_legged_msgs.msg import MotorStateArray
from go1_legged_msgs.msg import MotorState
from go1_legged_msgs.srv import SetControl

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

        self.state = MotorStateArray()

    def initialize_values_from_state(self, state: MotorStateArray):
        for i in range(12):
            self.joint_positions[i] = state.motor_state[i].q
            self.joint_velocities[i] = state.motor_state[i].dq
            self.joint_torques[i] = state.motor_state[i].tau

    def set_position(self, joint_nr: int, pos: float):
        self.joint_positions[joint_nr] = pos
        
    def set_velocity(self, joint_nr: int, vel: float):
        self.joint_velocities[joint_nr] = vel

    def set_torque(self, joint_nr: int, tau: float):
        self.joint_torques[joint_nr] = tau

    def set_joint_kp_kd(self, joint, kp, kd):
        self.joint_kps[joint] = kp
        self.joint_kds[joint] = kd

    def get_command(self):
        msg = JointCmd()
        msg.q = self.joint_positions
        msg.dq = self.joint_velocities
        msg.tau = self.joint_torques
        msg.Kp = self.joint_kps
        msg.Kd = self.joint_kds
        return msg

    def get_joint_data_point(self, timestep: int, joint_nr: int):
        return {"t": timestep, "q_is": self.state.motor_state[joint_nr].q, "q_set": self.joint_positions[joint_nr],
                "kp": self.joint_kps[joint_nr], "kd": self.joint_kds[joint_nr]}

    def motor_states_callback(self, state: MotorStateArray):
        self.state = state


def plot_datasets(data_sets, path="~/Documents/Recordings/autosave/"):
    fig, ax = plt.subplots()
    for i, data in enumerate(data_sets):
        measured_q = []
        set_q = []
        t = []
        for point in data:
            measured_q.append(point["q_is"])
            set_q.append(point["q_set"])
            t.append(point["t"])
        if i == 0:
            ax.plot(t, set_q, "b", label="step")
        ax.plot(t, measured_q, label=f"{data[0]['kp']}-{data[0]['kd']}")
    plt.show()
    # plt.savefig(f"{path}Kp{str(kp).replace('.', '-')}_Kd{str(kd).replace('.', '-')}.pdf", dpi='figure', format="pdf", backend="pgf")


def test(kp=5.0, kd=1.0):
    joint_controller = JointController()

    pub = rospy.Publisher("/base_node/joint_cmd", JointCmd, queue_size=1)
    rospy.Subscriber("/base_node/motor_states", MotorStateArray, joint_controller.motor_states_callback, queue_size=10)
    rospy.init_node("low_controller", anonymous=True)

    sample_multiplier = 2

    rate = rospy.Rate(500 * sample_multiplier)

    counter = 0
    dataset = []

    init_pos = -2.0
    end_pos = -1.5

    q0 = 0.0
    q1 = 1.2
    q2 = -2.0

    print(f"[i] Testing combination:{kp=}, {kd=}")
    joint_controller.set_joint_kp_kd(FR_2, kp, kd)
    joint_controller.set_velocity(FR_2, 2.0)

    joint_controller.set_torque(FR_0, -0.65)
    joint_controller.set_torque(FL_0, +0.65)
    joint_controller.set_torque(RR_0, -0.65)
    joint_controller.set_torque(RL_0, +0.65)

    for leg in leg_joints:
        for joint, q in zip(leg, [q0, q1, q2]):
            joint_controller.set_position(joint, q)

    if not rospy.is_shutdown():
        pub.publish(joint_controller.get_command())
        for _ in range(2000 * sample_multiplier):
            rate.sleep()

    while not rospy.is_shutdown():
        dataset.append(joint_controller.get_joint_data_point(counter, FR_2))
        counter += 1

        if counter <= 2000 * sample_multiplier:
            joint_controller.set_position(FR_2, init_pos)
        elif counter < 4000 * sample_multiplier:
            joint_controller.set_position(FR_2, end_pos)
        else:
            break

        pub.publish(joint_controller.get_command())

        rate.sleep()

    for leg in leg_joints:
        for joint, q in zip(leg, [q0, q1, q2]):
            joint_controller.set_position(joint, q)

    if not rospy.is_shutdown():
        pub.publish(joint_controller.get_command())
        for _ in range(2000 * sample_multiplier):
            rate.sleep()

    print("[i] Config done \n\n")
    return dataset


def main():
    rospy.wait_for_service("/base_node/set_control")
    try:
        set_control = rospy.ServiceProxy("/base_node/set_control", SetControl)
        response = set_control("Full")
        print(f"[i] Control mode selection {'success' if response.success else 'failed'}:\n\t{response.message}")
    except rospy.ServiceException as se:
        print(se)
        exit(-1)

    kps = list(np.arange(3.0, 7.0, 1.0))
    kds = list(np.arange(1.0, 2.0, 0.25))
    results = []
    for kp in kps:
        for kd in kds:
            results.append(test(kp, kd))

    plot_datasets(results)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        print(f"ROS Error: {e}\n\nExiting...")
