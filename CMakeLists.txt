cmake_minimum_required(VERSION 3.8)
project(multi_target_kf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/Detections.msg"
  "msg/KFTrack.msg"
  "msg/KFTracks.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs 
)

# Get the typesupport target for custom messages
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Hungarian algorithm library
add_library(hungarian src/hungarian.cpp)
ament_export_targets(hungarian HAS_LIBRARY_TARGET)
target_include_directories(hungarian
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Motion models library (header-only)
add_library(motion_models INTERFACE)
target_include_directories(motion_models
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_targets(motion_models HAS_LIBRARY_TARGET)

# KF tracker library
add_library(kf_tracker src/kf_tracker.cpp)
ament_target_dependencies(kf_tracker Eigen3)
target_link_libraries(kf_tracker 
  hungarian
  motion_models
  ${cpp_typesupport_target})
ament_export_targets(kf_tracker HAS_LIBRARY_TARGET)
target_include_directories(kf_tracker
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# ROS interface library
add_library(tracker_ros src/tracker_ros.cpp)
ament_target_dependencies(tracker_ros 
  rclcpp 
  Eigen3 
  geometry_msgs
  std_msgs
  sensor_msgs)
target_link_libraries(tracker_ros 
  kf_tracker
  ${cpp_typesupport_target})
ament_export_targets(tracker_ros HAS_LIBRARY_TARGET)
target_include_directories(tracker_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Tracker ROS node
add_executable(tracker_node src/tracker_node.cpp)
target_link_libraries(tracker_node tracker_ros)
target_include_directories(tracker_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install documentation
install(FILES
  README.md
  # README_MODELS.md  # Uncomment once you create this file
  DESTINATION share/${PROJECT_NAME}
)

# Install libraries
install(
  TARGETS kf_tracker
  EXPORT kf_tracker
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS hungarian
  EXPORT hungarian
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS motion_models
  EXPORT motion_models
  INCLUDES DESTINATION include
)

install(
  TARGETS tracker_ros
  EXPORT tracker_ros
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS tracker_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/simulate_circle.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add Google Test dependency
  find_package(ament_cmake_gtest REQUIRED)

  # Add the test executable
  ament_add_gtest(test_motion_models test/test_motion_models.cpp)
  
  # Link test dependencies
  target_link_libraries(test_motion_models
    kf_tracker
    hungarian
    Eigen3::Eigen
    ${cpp_typesupport_target}
  )
  
  # Include directories for tests
  target_include_directories(test_motion_models
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  # Install the test executable
  install(TARGETS test_motion_models
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()