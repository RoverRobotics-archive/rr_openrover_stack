^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rr_openrover_basic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2018-07-10)
------------------
* Ver 0.0.01  - subscribes to cmd_vel/managed with approximations for scaling factors. Removed joystick script and node.
* Ver 0.0.01 with joystick code
* added y-button toggle between analog stick control and heading control
* Deleted wheel_type
* Merge branch 'master' of https://github.com/RoverRobotics/rr_openrover_basic
* Removed unused ros params from openrover_node, added package.xml details
* Formatted Published topics and added more detail.
* Formatted Readme
  Still need to add more detail to Publishers, Subscribers, and Parameters
* First Readme additions
  Added general info to readme. Still need to look through joystick params to get an idea what they are used for.
* Changed exceptions to throw to main() level to decouple ROS from the C++ code. Operating at 10hz, 2hz, and 1hz with bandwidth for only about 8 more msgs/sec to be added.
* Fixed class method capitalizations, tested max messsage rate, added comments about max message rate.
* Code cleaning with code review changes made
* Functioning ros driver publishing raw data to raw_encoders, raw_med_rate_data, and raw_slow_rate_data
* Buffered serial read at max speed yielding ~55 write/reads per second. Polling Encoders at 10Hz, Medium Speed Robot Data at 2Hz, and Low Speed Robot data at 1Hz.
* Serial comms with 3 priority buffers is functioning. Started to add encoder publisher.
* mostly working driver without publishers. overloading serial so need to reduce read/write rate
* working low speed mode with joystick control. x_button toggle does not work but does not break anything
* Add flipper control
* Started adding reading of encoders but not seeing any data back for some reason
* Xbox control is working, still nee to prevent commands over 250 and below 0
* Node successfully compiles but subscriber isn't created. Once it is we should have XBOX control
* Fixed merge conflicts and tested example.launch and is working. It spins motos backwards but there is o way to stop it
* trying to fix merge conflicts Merge branch 'master' of https://github.com/RoverRobotics/rr_openrover_basic
* Working example script to spin motors backwards
* Initial commit, includes an example file that turns the motors
* Contributors: Erik Beall, Nick Fragale, jack, jack-digilabs, shoemakerlevy9
