#!/usr/bin/env python3
import rospy
from unitree_legged_msgs.msg import MotorCmd


def walk():
    high_pub = rospy.Publisher("/go1_gazebo/FL_calf_controller/command", MotorCmd, queue_size=1)
    rospy.init_node("walk_controller", anonymous=True)

    rate = rospy.Rate(10)

    cmd_msg = MotorCmd()
    cmd_msg.mode = 10
    cmd_msg.q = 1.0
    cmd_msg.dq = 0.01
    cmd_msg.tau = 0.0
    cmd_msg.Kp = 300.0
    cmd_msg.Kd = 15.0
    cmd_msg.reserve = [0, 0, 0]

    while not rospy.is_shutdown():
        rospy.loginfo(cmd_msg)
        high_pub.publish(cmd_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        walk()
    except rospy.ROSInterruptException as e:
        print(f"ROS Error: {e}\n\nExiting...")
