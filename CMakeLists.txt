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


# let the compiler search for headers in the include folder
include_directories(include)

# define a library target called my_value_converter_library
add_library(multi_target_kf src/kf_tracker.cpp)

# this line to exports the library
ament_export_targets(multi_target_kf HAS_LIBRARY_TARGET)

find_package(Eigen3 REQUIRED)
target_link_libraries(multi_target_kf Eigen3::Eigen)

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/multi_target_kf
  DESTINATION include
)

install(
  TARGETS multi_target_kf
  EXPORT multi_target_kf
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# add_executable(talker src/publisher_member_function.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# install(TARGETS
#   executable
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
