cmake_minimum_required(VERSION 2.8.3)
project(rr_openrover_driver)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  rr_openrover_driver_msgs
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
   rr_openrover_driver_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime rr_openrover_driver_msgs sensor_msgs ds4_driver
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(odom_control src/odom_control.cpp)

add_executable(openrover_driver_node src/openrover_driver.cpp)
add_dependencies(openrover_driver_node rr_openrover_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(openrover_driver_node ${catkin_LIBRARIES} odom_control)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS openrover_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS odom_control
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

catkin_install_python(PROGRAMS scripts/diagnostics.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)
