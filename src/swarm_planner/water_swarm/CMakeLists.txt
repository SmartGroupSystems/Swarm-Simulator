cmake_minimum_required(VERSION 3.0.2)
project(water_swarm)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  common_msgs
  cv_bridge
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
  ${PCL_INCLUDE_DIRS}
)
add_library(formation_control 
  src/EdtTransform.cpp
  src/formation_grad.cpp
  src/sph_visulization.cpp)
target_link_libraries(formation_control ${catkin_LIBRARIES})

# add_executable(neighbors src/neighbors.cpp)
# target_link_libraries(neighbors ${catkin_LIBRARIES})

# add_executable(sph_planner src/sph.cpp)
# target_link_libraries(sph_planner ${catkin_LIBRARIES})

# add_executable(sph_particles src/sph_particles.cpp)
# target_link_libraries(sph_particles ${catkin_LIBRARIES})

# add_executable(self_organize src/self_organize.cpp)
# target_link_libraries(self_organize ${catkin_LIBRARIES})

add_executable(sph_zhang src/sph_zhang.cpp)
target_link_libraries(sph_zhang ${catkin_LIBRARIES})

add_executable(sph_vis src/sph_vis.cpp)
target_link_libraries(sph_vis ${catkin_LIBRARIES})

add_executable(sph_tase src/sph_tase.cpp)
target_link_libraries(sph_tase formation_control ${catkin_LIBRARIES})

add_executable(sph_zhang_3d src/sph_zhang_3d.cpp)
target_link_libraries(sph_zhang_3d ${catkin_LIBRARIES})
