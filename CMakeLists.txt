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


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KFTrack.msg"
  "msg/KFTracks.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
# let the compiler search for headers in the include folder
# include_directories(include)

#
# hungarian.cpp
#
add_library(hungarian src/hungarian.cpp)
# this line to exports the library
ament_export_targets(hungarian HAS_LIBRARY_TARGET)
target_include_directories(hungarian
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

#
# kf_tracker.cpp
#

# define a library target called my_value_converter_library
add_library(kf_tracker src/kf_tracker.cpp)
ament_target_dependencies(kf_tracker Eigen3)
target_link_libraries(kf_tracker hungarian)

# this line to exports the library
ament_export_targets(kf_tracker HAS_LIBRARY_TARGET)

target_include_directories(kf_tracker
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# The following to be able to use the custom msgs in the same pkg
rosidl_target_interfaces(kf_tracker
                        ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target_link_libraries(multi_target_kf Eigen3::Eigen)


add_executable(tracker_node src/tracker_node.cpp)
ament_target_dependencies(tracker_node rclcpp Eigen3)
target_link_libraries(tracker_node kf_tracker)
target_include_directories(tracker_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(tracker_node
${PROJECT_NAME} "rosidl_typesupport_cpp")
# ament_target_dependencies(talker rclcpp std_msgs)

#
# Install
#

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

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

# Install target
install(TARGETS tracker_node
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
