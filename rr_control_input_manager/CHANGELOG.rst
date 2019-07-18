^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rr_control_input_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.2 (2019-07-18)
-----------
* if there are test failures then exit set exit code to 1 to cause failure in circleci
* testing failures
* Merge pull request `#4 <https://github.com/RoverRobotics/rr_openrover_ros1/issues/4>`_ from RoverRobotics/feature/enable-circlecli-testing
  Feature/enable circlecli testing
* added circleci to build
* Merge pull request `#2 <https://github.com/RoverRobotics/rr_openrover_ros1/issues/2>`_ from RoverRobotics/feature/add_twist_mux_support
  Feature/add twist mux support
* removed e-stop from mux and mapped x and y buttons to mux
* fixing style and discription issues from review.
* fixed to only support xboxdrv and to communicate issues with wired controllers
* made changes recommended by Dan, and prepping xbox_mapper.py to support wired version of xbox controller
* made a number of fixes recommended by Dan.  Also adjusted tests to reflect that a new instance for each test class is created for for each test method
* changed rostest to test_depend
* removing addition unneeded comments
* fixed alignment issue
* added licenses to packages and metapackage, CHANGELOG not added to metapackage yet
* adding back Twist Mux to teleop.launch after accidentally removing
* fixed missing configs from install
* fixing missed python scripts to be installed
* Pushing comments
* commenting
* fixed teleop script to reflect desired turn rate
* fixed teleop script to reflect desired turn rate
* fixed teleop script to reflect desired turn rate
* moved init to top of file
* moved init to top of file
* moved init to top of file
* fixed duration tests for toggle
* fixed duration tests for toggle
* pushing openrover_basic and xbox_mapper fixes
* pushing testing changes
* pushing test progress
* fixing stale file reference in example.launch
* refactored control_input_manager from a mux to a node that test how out of sync a TwistStamp message is
* merging rr_control_input_manager into repo
* Contributors: padiln

0.7.1 (2019-06-24)
------------------

0.7.0 (2019-05-13)
------------------

0.6.2 (2019-01-23)
------------------

0.6.1 (2018-12-20)
------------------

0.6.0 (2018-12-07)
------------------

0.5.1 (2018-09-20)
------------------

0.5.0 (2018-09-06)
------------------

0.4.0 (2018-07-23)
------------------

0.3.0 (2018-07-19)
------------------

0.2.0 (2018-07-17)
------------------

0.1.0 (2018-07-10)
------------------
