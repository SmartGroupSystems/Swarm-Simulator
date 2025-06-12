cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
  message_generation
  message_runtime
  common_msgs
  visualization_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

# 确保消息生成在编译之前
add_message_files(
  FILES
  dynamic_obs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  common_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs common_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(random_forest src/random_forest_sensing.cpp)
target_link_libraries(random_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dynamic_forest src/dynamic_forest_sensing.cpp)
target_link_libraries(dynamic_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dynamic_forest_seq src/dynamic_forest_seq_sensing.cpp)
target_link_libraries(dynamic_forest_seq
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})
  

add_executable(click_map src/click_map.cpp)
target_link_libraries(click_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
  
add_executable(click_map_3d src/click_map_3d.cpp)
  target_link_libraries(click_map_3d
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable(move_base_goal_pub src/move_base_goal_pub.cpp)
  target_link_libraries(move_base_goal_pub
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

    add_executable(click_map_dynamic src/click_map_dynamic.cpp)
    add_dependencies(click_map_dynamic ${PROJECT_NAME}_generate_messages_cpp)
    target_link_libraries(click_map_dynamic
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES})
    
    add_executable(dyn_map src/dyn_map.cpp)
    add_dependencies(dyn_map ${PROJECT_NAME}_generate_messages_cpp)
    target_link_libraries(dyn_map
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES})
    
    add_executable(static_dyn_map src/static_dyn_map.cpp)
    add_dependencies(static_dyn_map ${PROJECT_NAME}_generate_messages_cpp)
    target_link_libraries(static_dyn_map
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES})
    
  
add_executable (map_recorder src/map_recorder.cpp )
target_link_libraries(map_recorder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable (local_sensing src/local_sensing.cpp )
  target_link_libraries(local_sensing
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable (random_map src/random_map.cpp )
target_link_libraries(random_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable (map_pub src/map_publisher.cpp )
target_link_libraries(map_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable (exp_map src/exp_map.cpp )
target_link_libraries(exp_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable (map2bt src/map2bt.cpp )
  target_link_libraries(map2bt
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

install(TARGETS random_forest click_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
