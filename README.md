# go1controller package
The `go1controller` package can be used to control the robot via ROS.
This package contains the following executable ROS nodes:
+ `testHigh.py`: Can be used to test the general connection by cycling between standing and laying down.
+ `joystic_control`: Provides a translation from `sensor_msgs:joy` to `high_cmd` messages to control the robot.
+ `robot_state.py`: Prints out interesting values from the `high_state` topic.
+ `highstate_to_tf2`: Converts the HighState messages to tf2 messages to display the robot and its feet in their respective positions  
+ `highstate_to_imu`: Converts the HighState messages to imu messages to display the robots orientation and acceleration
+ `highstate_to_range`: Converts the HighState messages to range messages to display the measured ranges of the three ultrasonic sensors

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

### highstate_to_tf2
To use the positional data in RViz execute the following in one terminal:
```bash
rosrun go1controller highstate_to_tf2
```
If not already done, start RViz in another terminal:
```bash
rviz
```
By clicking in the "Add" button in the bottom left corner and selecting "tf" from the list, you can add the transform view to your window.
Make sure to set the `Fixed Frame` in the global options to "world" (this option only appears, if data was published on the `/tf` topic).

### highstate_to_imu
To use the imu data in RViz execute the following in one terminal:
```bash
rosrun go1controller highstate_to_imu
```
If not already done, start RViz in another terminal:
```bash
rviz
```
By clicking in the "Add" button in the bottom left corner and selecting "imu" from the list, you can add the imu view to your window.

### highstate_to_range
To use the range data in RViz execute the following in one terminal:
```bash
rosrun go1controller highstate_to_range
```
If not already done, start RViz in another terminal:
```bash
rviz
```
By clicking in the "Add" button in the bottom left corner and selecting "Range" from the list, you can add the Range view to your window.
Since the robot has three ultrasonic sensors, you have to add all three of them and then choose each of the three topics once.


## Troubleshooting
If the robot doesn't move, make sure you are connected via ethernet or Wi-Fi.
Make sure, you can ping `192.168.123.161`.

To change the joystick input device follow [this link](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
