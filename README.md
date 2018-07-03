This package lets you control the OpenRover Basic using ROS

Prerequisites

-ROS Kinetic or above
-Ubuntu 16.04 or above

Install
cd ~catkin_ws/src
git clone https://github.com/RoverRobotics/rr_openrover_basic.git
cd ~catkin_ws
catkin_make
source devel/setup.bash

Running Examples
roslaunch rr_openrover_basic example.launch

Published Topics:
/raw_encoders
/raw_med_rate_data
/raw_slow_rate_data

Subscribed Topics:
/cmd_vel: Subscribes to /teleop_twist_joy and /joystick used to command wheels.
/joystick/x_button: Subscribes to std_msgs/Bool to switch between low speed mode and normal mode.
/rosout

openrover_basic_node Parameters:
"port": The /dev/ttyUSB# of the FTDI converter

Xbox Controller Mapping Parameters:
"max_vel": scalar used in calculating the maximum velocity when using an xbox controller
"drive_type": 
"full_throttle":
"adjustable_throttle":
"increment":

joy_node Parameters:
"autorepeat_rate": default value = 10
