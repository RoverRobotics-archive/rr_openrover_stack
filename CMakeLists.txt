cmake_minimum_required(VERSION 2.8.3)
project(rr_openrover_basic)

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(odom_control src/odom_control.cpp)

add_executable(openrover_basic_node src/openrover_basic.cpp)
add_dependencies(openrover_basic_node rr_openrover_basic_generate_messages_cpp)
target_link_libraries(openrover_basic_node ${catkin_LIBRARIES} odom_control)

