#!/usr/bin/env python3
import rospy
from unitree_legged_msgs.msg import HighCmd

HIGHLEVEL = 0xEE
LOWLEVEL = 0xFF
TRIGERLEVEL = 0xF0


def init_high_command() -> HighCmd:
    cmd_msg = HighCmd()
    cmd_msg.head = [0xFE, 0xEF]
    cmd_msg.levelFlag = HIGHLEVEL
    cmd_msg.mode = 0
    # 0. idle, default stand
    # 1. force stand (controlled by dBodyHeight + ypr)
    # 2. target velocity walking (controlled by velocity + yawSpeed)
    # 3. target position walking (controlled by position + ypr[0])
    # 4. path mode walking (reserve for future release)
    # 5. position stand down.
    # 6. position stand up
    # 7. damping mode
    # 8. recovery stand
    # 9. backflip
    # 10. jumpYaw
    # 11. straightHand
    # 12. dance1
    # 13. dance2

    cmd_msg.gaitType = 0
    # 0.idle
    # 1.trot
    # 2.trot running
    # 3.climb stair
    # 4.trot obstacle

    cmd_msg.speedLevel = 0
    # 0. default low speed.
    # 1. medium speed
    # 2. high speed. during walking,
    # only respond MODE 3

    cmd_msg.footRaiseHeight = 0
    # (unit: m, default: 0.08m), foot up height while walking, delta value

    cmd_msg.bodyHeight = 0
    # (unit: m, default: 0.28m), delta value

    cmd_msg.euler = [0, 0, 0]
    # (unit: rad), roll pitch yaw in stand mode

    cmd_msg.velocity = [0.0, 0.0]
    # (unit: m/s), forwardSpeed, sideSpeed in body frame

    cmd_msg.yawSpeed = 0.0
    # (unit: rad/s), rotateSpeed in body frame

    cmd_msg.reserve = 0

    return cmd_msg


def test():
    high_pub = rospy.Publisher("/high_cmd", HighCmd, queue_size=1)
    rospy.init_node("walk_controller", anonymous=True)

    rate = rospy.Rate(10)

    cmd_msg = init_high_command()

    counter = 0

    while not rospy.is_shutdown():

        counter = (counter + 1) % 100
        cmd_msg.mode = 5 if counter >= 50 else 6

        rospy.loginfo(cmd_msg)
        high_pub.publish(cmd_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException as e:
        print(f"ROS Error: {e}\n\nExiting...")
