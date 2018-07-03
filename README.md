# rr_openrover_basic
This package lets you control the OpenRover Basic using ROS

## Prerequisites
-ROS Kinetic or above  
-Ubuntu 16.04 or above  

## Install
cd ~catkin_ws/src  
git clone https://github.com/RoverRobotics/rr_openrover_basic.git  
cd ~catkin_ws  
catkin_make  
source devel/setup.bash  

## Running Examples
roslaunch rr_openrover_basic example.launch

## Published Topics:
* `/raw_encoders`: Publishes `rr_openrover_basic/RawRrOpenroverBasicFastRateData` custom message that contains the left, right, and flipper motor raw encoder values at 10hz as well as a header.
* `/raw_med_rate_data`:  Publishes `rr_openrover_basic/RawRrOpenroverBasicMedRateData` custom message at 2Hz that contains the following data: header, reg_pwr_total_current, reg_motor_fb_rpm_left, reg_motor_fb_right, reg_flipper_fb_position_pot1, reg_flipper_fb_position_pot2, reg_motor_fb_current_left, reg_motor_fb_current_right, reg_motor_charger_state, reg_power_a_current, reg_power_b_current, reg_motor_flipper_angle  
* `/raw_slow_rate_data`: Publishes `rr_openrover_basic/RawRrOpenroverBasicSlowRateData` at 1Hz custom message that contains the following data: header, reg_motor_fault_flag_left, reg_motor_temp_left, reg_motor_temp_right, reg_power_bat_voltage_a, reg_power_bat_voltage_b, reg_robot_rel_soc_a, reg_robot_rel_soc_b, buildno

## Subscribed Topics:
* `/cmd_vel`:
  Subscribes to /teleop_twist_joy and /joystick used to command wheels.
* `/joystick/x_button`:
  Subscribes to std_msgs/Bool to switch between low speed mode and normal mode.
* `/rosout`:

## openrover_basic_node Parameters:
* `port`: The /dev/ttyUSB# of the FTDI converter. Typicall /dev/ttyUSB1

## Xbox Controller Mapping Parameters:  
* `max_vel`: scalar used in calculating the maximum velocity when using an xbox controller*`"drive_type`. Currently unused.
* `full_throttle`: Currently unused.
* `adjustable_throttle`: Currently unused.
* `increment`: Currently unused.

## joy_node Parameters:
* `autorepeat_rate`: default value = 10. Currently unused.
