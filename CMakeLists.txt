cmake_minimum_required(VERSION 3.0.2)
project(multi_target_kf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  KFTrack.msg
  KFTracks.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  multi_target_kf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

#
# KF node
#
add_library(kf_tracker src/kf_tracker.cpp)
add_dependencies(kf_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kf_tracker ${catkin_LIBRARIES})

add_executable(kf_tracker_node src/kf_tracker_node.cpp)
add_dependencies(kf_tracker_node kf_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kf_tracker_node kf_tracker ${catkin_LIBRARIES})