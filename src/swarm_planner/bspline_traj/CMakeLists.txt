cmake_minimum_required(VERSION 3.0.2)
project(bspline_race)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g")

# add_definitions(-std=c++11 -g -Og -ftree-vectorize -ffast-math -march=native -lpthread)
find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  visualization_msgs
  message_generation
  message_filters
  cv_bridge
  common_msgs
  plan_env
  path_searching
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_race
  CATKIN_DEPENDS roscpp rospy tf std_msgs visualization_msgs message_runtime common_msgs plan_env path_searching
  DEPENDS message_generation
)
include_directories(
  SYSTEM
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${NLOPT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/devel/include
)

# 定义库 (race和3d版本)
add_library(bspline_race
   src/UniformBspline.cpp
   src/bspline_opt.cpp 
   src/bspline_race.cpp
   src/gvf.cpp
)
target_link_libraries(bspline_race ${catkin_LIBRARIES})

add_library(bspline_race_3d
   src/UniformBspline_3d.cpp
   src/bspline_opt_3d.cpp 
   src/bspline_race_3d.cpp
   src/gvf_manager.cpp
   src/gvf.cpp
)
target_link_libraries(bspline_race_3d ${catkin_LIBRARIES})

# 可执行文件
add_executable(flag_planner src/test_planning.cpp)
target_link_libraries(flag_planner 
    bspline_race
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARIES}
)

# swarm planning (2D版本)
add_executable(swarm_planning src/swarm_planning.cpp)
target_link_libraries(swarm_planning 
    bspline_race
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARIES}
)

# swarm planning (3D版本)
add_executable(swarm_planning_3d src/swarm_planning_3d.cpp)
target_link_libraries(swarm_planning_3d 
    bspline_race_3d
    ${catkin_LIBRARIES}
    ${NLOPT_LIBRARIES}
)

add_executable( formation_planning 
    src/formation_planning.cpp
 )
target_link_libraries(formation_planning 
bspline_race_3d
 ${catkin_LIBRARIES}
 ${NLOPT_LIBRARIES}
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})  
add_executable(traj_saver src/traj_saver.cpp)
target_link_libraries(traj_saver ${catkin_LIBRARIES})  
