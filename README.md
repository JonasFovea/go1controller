# go1controller package
The `go1controller` package can be used to control the robot via ROS.
It currently supports testing the connection by cycling between standing and laying down with the `testHigh.py` script and controlling the robot with a gamepad.

## Usage

### testHigh
To run the `testHigh.py` script open a terminal and run:
```bash 
roslaunch unitree_legged_real real.launch ctrl_level:=HIGHLEVEL
```
This starts the translation layer between ROS messages and the UDP protocol.

Then start up a second terminal and run:
```bash
rosrun go1controller testHigh.py
```
The robot should now change its position every ten seconds.

### joystick_control
To run the `joystick_control.py` script you can either use the steps above and replace the script name or run the provided launch file:
```bash
roslaunch go1coltroller joystick_control.launch
```

Then in another terminal start the joystick input:
```bash
rosrun joy joy_node
```
Make sure a gamepad is connected to the computer. You maybe also have to change the selected device.

## Troubleshooting
If the robot doesn't move, make sure you are connected via ethernet or Wi-Fi.
Make sure, you can ping `192.168.123.161`.

To change the joystick input device follow [this link](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
