cmake_minimum_required(VERSION 3.0.2)
project(water_swarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  common_msgs
)

# add_message_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs roscpp rospy common_msgs
)

include_directories(
  include/water_swarm
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${NLOPT_INCLUDE_DIRS}
)

add_executable(neighbors src/neighbors.cpp)
target_link_libraries(neighbors ${catkin_LIBRARIES})

add_executable(sph_planner src/sph.cpp)
target_link_libraries(sph_planner ${catkin_LIBRARIES})

add_executable(sph_particles src/sph_particles.cpp)
target_link_libraries(sph_particles ${catkin_LIBRARIES})

add_executable(sph_zhang src/sph_zhang.cpp)
target_link_libraries(sph_zhang ${catkin_LIBRARIES})