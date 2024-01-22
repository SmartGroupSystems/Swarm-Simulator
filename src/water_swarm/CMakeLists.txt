cmake_minimum_required(VERSION 3.0.2)
project(water_swarm)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs roscpp rospy
)

include_directories(
  include/water_swarm
  ${catkin_INCLUDE_DIRS}
)

add_executable(neighbors src/neighbors.cpp)
target_link_libraries(neighbors ${catkin_LIBRARIES})


