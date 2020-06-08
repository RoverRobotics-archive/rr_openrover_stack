^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rr_openrover_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2020-06-08)
------------------

0.8.0 (2020-03-25)
------------------

0.7.4 (2020-02-19)
------------------
* changed maintainership of all packages to nick padilla
* Hotfix, remove integral gain overwrite
* fix variable setting error affecting PID
* exposed PID gains as ros parameters
* added autodocking launch file and descriptions
* Adding description and 4WD code
* The coefficient that scaled the flipper motor commands wasn't being initialized properly so the flipper motor commands sent to the robot were flicking all over the place with the smallest change in commanded velocity. Solution: initialize the motor speed coefficients in every wheel configuration.
* fix rover message buffer race condition
* Updating the openerrover_driver to clean up how rover data is published. The current state of the driver data read from the rover is categorized as fast, medium, and slow rate data. All data types for a given rate are published in the same message. This requires users to parse the topic message to find the field for the desired information in the message. This pull request eliminates the rate based messages and instead publishes odometry, battery_state_of_charge and separate topics. For backwards compatibility the boolean parameter "use_legacy" publishes the rate topics if set to true.
  Additionally, all uses of tf have been updated to use tf2.
* Contributors: Jack Kilian, padiln

0.7.3 (2019-10-14)
------------------
* Migrating all usage of tf to tf2
* Simplified rover data messages and only publish rate-based data in legacy mode

0.7.2 (2019-07-18)
------------------
* Changed name to rr_openrover_drive from rr_openrover_basic
* Refactored openrover_driver to accept Twist msgs instead of TwistStamped msgs

0.7.1 (2019-6-24)
------------------
* Changed default port to `/dev/rover`
* Changed CMAke to install `diagnostics.py`

0.7.0 (2019-5-13)
------------------
* Fixed issue #12
* Added fan speed control
* Added launch file for 2wd teleop
* Changed default fast rate data to 10hz

0.6.2 (2019-1-23)
------------------
* Fixed issue #13

0.6.1 (2018-12-13)
------------------
* Migrated release repo from /roverrobotics to /roverrobotics-release
* Changed rosparam slippage_factor to traction_factor
* Changed default weight to 20.0 lbs
* Changed default drive type to 4WD
* Changed default closed_loop_control_on to false
* Removed low_speed_mode

0.6.0 (2018-12-08)
------------------
* Added ROS topic /rr_openrover_driver/battery_status_a
* Added ROS topic /rr_openrover_driver/battery_status_b

0.5.1 (2018-12-07)
------------------
* Added ROS topic /rr_openrover_driver/motor_speeds_commanded
* Added ROS topic /rr_openrover_driver/charging
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
* Added ROS topic /rr_openrover_driver/odom_encoder
* Added 2nd maintainer to package.xml and URL to wiki

0.2.0 (2018-07-17)
------------------
* Fixed low-speed mode turning so it has a continuous turning speed range
* Added dependency to generate messages cpp to CMakeLists
* updated package.xml to new format

0.1.0 (2018-07-10)
------------------
* first public release for Kinetic
