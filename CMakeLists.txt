cmake_minimum_required(VERSION 2.8.3)
project(rr_openrover_basic)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  tf
)

add_message_files(
   FILES
   RawRrOpenroverBasicFastRateData.msg
   RawRrOpenroverBasicMedRateData.msg
   RawRrOpenroverBasicSlowRateData.msg
   SmartBatteryStatus.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(odom_control src/odom_control.cpp)

add_executable(openrover_basic_node src/openrover_basic.cpp)
add_dependencies(openrover_basic_node rr_openrover_basic_generate_messages_cpp)
target_link_libraries(openrover_basic_node ${catkin_LIBRARIES} odom_control)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS openrover_basic_node 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

