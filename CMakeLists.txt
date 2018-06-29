cmake_minimum_required(VERSION 2.8.3)
project(rr_openrover_basic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  message_generation
)

add_message_files(
   FILES
   RawRrOpenroverBasic.msg
   RawRrOpenroverBasicFastRateData.msg
   RawRrOpenroverBasicMedRateData.msg
   RawRrOpenroverBasicSlowRateData.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(openrover_basic_node src/openrover_basic.cpp)

target_link_libraries(openrover_basic_node ${catkin_LIBRARIES} )
