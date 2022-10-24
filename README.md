# go1controller package
The `go1controller` package can be used to control the robot via ROS.
This package contains the following executable ROS nodes:
+ `testHigh.py`: Can be used to test the general connection by cycling between standing and laying down.
+ `joystic_control`: Provides a translation from `sensor_msgs:joy` to `high_cmd` messages to control the robot.
+ `robot_state.py`: Prints out interesting values from the `high_state` topic.

Additionally, this package contains a launch file to start both `unitree_legged_real real.launch` and `joystick_control`. 

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
To run the `joystick_control` program you can either start the translation layer by unitree as shown above 
and then run `rosrun go1controller joystick_control [gamepad layout number]`
or run the provided launch file, which also starts the translation layer:
```bash
roslaunch go1coltroller joystick_control.launch [gamepad layout number]
```

Then in another terminal start the joystick input:
```bash
rosrun joy joy_node
```
Make sure a gamepad is connected to the computer. You maybe also have to change the selected device.

### robot_state
To view selected fields of the received messages on the `high_state`-topic simply run the following:
```bash
rosrun go1controller robot_state.py
```


## Troubleshooting
If the robot doesn't move, make sure you are connected via ethernet or Wi-Fi.
Make sure, you can ping `192.168.123.161`.

To change the joystick input device follow [this link](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
