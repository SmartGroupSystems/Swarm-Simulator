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
  OdomWithNeighbors.msg
  Odom.msg
  Position.msg
  Velocity.msg
  Acceleration.msg
  Force.msg
  OdomBroadcast.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs roscpp rospy
)

include_directories(
  include/water_swarm
  ${catkin_INCLUDE_DIRS}
)

add_executable(neighbors src/neighbors.cpp)
target_link_libraries(neighbors ${catkin_LIBRARIES})

add_executable(sph_planner src/sph.cpp)
target_link_libraries(sph_planner ${catkin_LIBRARIES})
