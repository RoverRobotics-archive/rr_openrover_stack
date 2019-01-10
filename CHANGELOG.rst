^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rr_openrover_basic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.6.1 (2018-12-13)
------------------
* Migrated release repo from /roverrobotics to /roverrobotics-release
* Changed rosparam slippage_factor to traction_factor
* Changed default weight to 20.0 lbs
* Changed default drive type to 4WD
* Changed default closed_loop_control_on to false
* Removed low_speed_mode

0.6.0 (2018-12-11)
------------------
* Added /rr_openrover_basic/battery_status_a
* Added /rr_openrover_basic/battery_status_b

0.6.0 (2018-12-08)
------------------
* Added ROS topic /rr_openrover_basic/battery_status_a
* Added ROS topic /rr_openrover_basic/battery_status_b

0.5.1 (2018-12-07)
------------------
* Added ROS topic /rr_openrover_basic/motor_speeds_commanded
* Added ROS topic /rr_openrover_basic/charging
* Fixed issue #8

0.5.0 (2018-12-07)
------------------
* Added ROS param for 'odom covariances'
* Added ROS param for 'slippage factor'
* Added diagnostics node
* Fixed issue #1

0.4.0 (2018-07-23)
------------------
* Added ROS param 'drive_type'

0.3.0 (2018-07-19)
------------------
* Added ROS topic /rr_openrover_basic/odom_encoder
* Added 2nd maintainer to package.xml and URL to wiki

0.2.0 (2018-07-17)
------------------
* Fixed low-speed mode turning so it has a continuous turning speed range
* Added dependency to generate messages cpp to CMakeLists
* updated package.xml to new format

0.1.0 (2018-07-10)
------------------
* first public release for Kinetic
