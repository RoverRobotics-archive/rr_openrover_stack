cmake_minimum_required(VERSION 2.8.3)
project(rr_rover_zero_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  diagnostic_msgs
  nav_msgs
  std_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rr_control_input_manager
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)


#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/rover_zero.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#catkin_install_python(PROGRAMS driver/roboclaw.py
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/driver
#)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Copy config files
#install(DIRECTORY config/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#  FILES_MATCHING PATTERN "*.yaml"
#)

## Copy test files
#install(DIRECTORY test/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/driver
#  FILES_MATCHING PATTERN "*"
#)

#############
## Testing ##
#############

