^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rr_rover_zero_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
2.0.1 (2020-12-14)
------------------

2.0.0 (2020-11-10)
------------------

1.2.0 (2020-11-05)
------------------

1.1.1 (2020-06-15)
------------------
* correct find_packge for rover zero cmakelists
* fixing pyKDL RPY to Quat conversion
* fix rospy dependency issue
* proper depend tag
* add pyserial to package.xml
* Contributors: 1102, padiln

1.1.0 (2020-06-08)
------------------
* release to melodic

1.0.0 (2020-06-08)
------------------
* add all ros parameters to launch files
* updated rr_rover_zero_driver to support pid and encoders
* merge with develop branch
* Merge pull request `#75 <https://github.com/RoverRobotics/rr_openrover_stack/issues/75>`_ from RoverRobotics/feature/rook/rover_zero_driver_estop
  Feature/rook/rover zero driver estop
* Fixed estop merge issue
* changed lock name to be more descriptive
* Merge pull request `#72 <https://github.com/RoverRobotics/rr_openrover_stack/issues/72>`_ from RoverRobotics/feature/rookrobotic/rover_zer_pid_get_set
  fix syntax issues
* Merge branch 'develop' into feature/rookrobotic/rover_zer_pid_get_set
* fix syntax issues
* Update rover_zero.py
  make _v_pid_overwrite a boolean.
* added odometry data to rover_zero.py
* Estop Handler in Roboclaw
* fix variable names
* getter and setter for Velocity Pid
* start odometry callback
* Default Twist message subscriber to /cmd_vel
* Contributors: William Rook, padiln

0.8.0 (2020-03-25)
------------------
* Added rr_rover_zero_driver

0.7.4 (2020-02-19)
------------------

0.7.3 (2019-10-14)
------------------

0.7.2 (2019-07-18)
------------------
