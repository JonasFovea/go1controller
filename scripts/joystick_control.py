#!/usr/bin/env python3
import math
import os
import re
import sys

import rospy
import yaml

from unitree_legged_msgs.msg import HighCmd
from sensor_msgs.msg import Joy


class JoyControl:
    __XBOX_WIRED = {"name": "XBOX Wired Controller (default)", "speed": 5, "angular": 3, "dirLR": 0, "dirFB": 1, "A": 0,
                    "B": 1}  # Mapping like 5.3 XBOX 360 Wired Controller
    __LOGITECH_F710 = {"name": "Logitech F710 Controller (default)", "speed": 7, "angular": 3, "dirLR": 0, "dirFB": 1,
                       "A": 1,
                       "B": 2}  # Mapping like 5.4 Logitech Wireless Gamepad F710
    __LOGITECH_EXTREME_3D = {"name": "Logitech Extreme 3D PRO (default)", "speed": 3, "angular": 2, "dirLR": 0,
                             "dirFB": 1, "A": 2, "B": 3}
    __MAPS = [__XBOX_WIRED, __LOGITECH_F710, __LOGITECH_EXTREME_3D]

    __SPEED_PLACES = 2
    __ANGLE_PLACES = 4
    __ANGULAR_PLACES = 2
    __DEADZONE_RADIUS = 0.025

    def __init__(self, map_select=0, speed_trigger=False) -> None:
        self.speed_trigger = speed_trigger
        self.velocity_unit = 1.0
        self.yaw_unit = 2.0

        # self.update_mappings()

        if len(self.__MAPS) > map_select >= 0:
            self.__map = self.__MAPS[map_select]
        else:
            print(f"[!]\tERROR: Map {map_select} not existing! Fallback to default (map 0).")
            self.__map = self.__MAPS[0]

        print(f"[i]\tSelected mapping: {self.__map}")

        rospy.init_node('joystick_control', anonymous=False)
        rospy.Subscriber("joy", Joy, self.__callback)
        self.pub = rospy.Publisher('high_cmd', HighCmd, queue_size=1)

    def __callback(self, joy_data: Joy) -> None:
        if not rospy.is_shutdown():
            axes = joy_data.axes
            buttons = joy_data.buttons
            try:
                speed_val = axes[self.__map["speed"]]
                angular_val = axes[self.__map["angular"]]
                dir_lr_val = axes[self.__map["dirFB"]]
                dir_fb_val = axes[self.__map["dirLR"]]

                msg = init_high_command()
                msg.mode = 2

                msg.velocity = self.__calc_velocity(self.__check_deadzone(dir_lr_val),
                                                    self.__check_deadzone(dir_fb_val),
                                                    self.__check_deadzone(speed_val))
                msg.yawSpeed = angular_val * self.yaw_unit

                self.pub.publish(msg)
            except IndexError:
                print(f"[!]\tERROR: Index out of range! \n\tMapping incorrect; "
                      f"please check if the selected mapping corresponds to the gamepad output.")
                rospy.signal_shutdown("Incorrect Mapping")
                exit()

    def __check_deadzone(self, val: float) -> float:
        return val if abs(val) > self.__DEADZONE_RADIUS else 0

    def __calc_angle(self, x: float, y: float) -> float:
        ang = math.atan2(self.__check_deadzone(y), self.__check_deadzone(x))  # - math.pi / 2
        ang = 0 if x == 0 and y == 0 else ang
        return round(ang, self.__ANGLE_PLACES)

    def __calc_velocity(self, x, y, trigger) -> tuple:
        if self.speed_trigger:
            speed = abs(round((trigger - 1) / -2, self.__SPEED_PLACES))
            speed = speed if speed <= 1.0 else 1.0
            return speed * x * self.velocity_unit, speed * y * self.velocity_unit
        else:
            return x * self.velocity_unit, y * self.velocity_unit

    @classmethod
    def update_mappings(cls, config_dir="config/") -> None:

        if os.path.isdir(config_dir):
            dir_list = os.listdir(config_dir)
            for file in dir_list:
                if re.match(".*\.yaml", file):
                    with open(config_dir + file) as config_file:
                        device = yaml.load(config_file, Loader=yaml.FullLoader)
                        try:
                            for index, existing_map in enumerate(cls.__MAPS):
                                if existing_map["name"] == device["name"]:
                                    cls.__MAPS[index] = device
                                    break
                            if device not in cls.__MAPS:
                                cls.__MAPS.append(device)
                        except KeyError:
                            # print(f"[!] Mapping invalid! Skipping configuration file: {file}")
                            pass

    @classmethod
    def get_mapping_names(cls) -> tuple:
        return tuple(mapping["name"] if "name" in mapping else "unnamed" for mapping in cls.__MAPS)

    @staticmethod
    def listener() -> None:
        rospy.spin()


def path_dialog() -> int:
    while True:
        path = input(f"[-p]\tEnter valid path to your config directory from here: {os.getcwd()}\n\t-q to quit: ")
        if path == "-q":
            return 0
        if os.path.isdir(path):
            break

    JoyControl.update_mappings(config_dir=path)
    print(f"[-p] updated maps from config path")
    print("Available gamepad maps:")
    maps = list(JoyControl.get_mapping_names())
    for i, name in enumerate(maps):
        print(f"\t{i:02d} {name}")
    while True:
        try:
            num = abs(int(input("Select valid gamepad index: ")))
            if num < len(list(maps)):
                return num
        except ValueError:
            pass

def init_high_command() -> HighCmd:
    HIGHLEVEL = 0xEE
    LOWLEVEL = 0xFF
    TRIGERLEVEL = 0xF0


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

if __name__ == '__main__':
    gamepad = 0
    speed_trigger = True
    if len(sys.argv) > 1:
        for arg in sys.argv[0:]:
            if str(arg) == "-h":
                print("\n\t\t\t HELP joystick_control.py")
                print("Usage: joystick_control.py [gamepad map index, speed input mode, configuration dialog]")
                print("\tgamepad map:\t\tselection of different gamepad types (int value)")
                print("\tspeed mode:\t\tselection of speed input right trigger or stick movement "
                      "\n\t\t\t\t(default: trigger, '-s' for stick)")
                print("\tconfiguration path:\tselection of config directory path\n\t\t\t\t('-p' to enter dialog for "
                      "configuration selection)")
                print("Available gamepad maps:")
                for i, name in enumerate(JoyControl.get_mapping_names()):
                    print(f"\t{i:02d} {name}")
                exit(0)
            # print("[i] Starting converter from /joy to /HEX_move")
            elif str(arg) == "-s":
                speed_trigger = False
            elif str(arg) == "-p":
                gamepad = path_dialog()

            else:
                try:
                    gamepad = abs(int(arg))
                except ValueError:
                    gamepad = 0
            # if len(sys.argv) > 2:
            #     if str(sys.argv[2]) == "-s":
            #         speed_trigger = False
        print(f"[i]\tArgs:{sys.argv[0:]}")
        print(f"[i]\tSelection: Gamepad type {gamepad}, Speed input {'Trigger' if speed_trigger else 'Stick'}")

    converter = JoyControl(map_select=gamepad, speed_trigger=speed_trigger)
    converter.listener()
