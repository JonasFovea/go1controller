#!/usr/bin/env python3
import rospy
from unitree_legged_msgs.msg import HighCmd


def walk():
    high_pub = rospy.Publisher("/go1_controller/high_control", HighCmd, queue_size=1)
    rospy.init_node("walk_controller", anonymous=True)

    rate = rospy.Rate(10)

    cmd_msg = HighCmd()
    cmd_msg.levelFlag = 0x00
    cmd_msg.mode = 2                # Standing: 1; Walking: 2
    cmd_msg.gaitType = 1
    cmd_msg.speedLevel = 0
    cmd_msg.footRaiseHeight = 0.08  # m default 0.08m
    cmd_msg.bodyHeight = 0.28       # m default 0.28m
    cmd_msg.velocity = [0.1, 0]     # m/s forwardSpeed, sideSpeed
    cmd_msg.yawSpeed = 0            # rad/s


    while not rospy.is_shutdown():
        rospy.loginfo(cmd_msg)
        high_pub.publish(cmd_msg)
        rate.sleep()




if __name__ == "__main__":
    try:
        walk()
    except rospy.ROSInterruptException as e:
        print(f"ROS Error: {e}\n\nExiting...")
