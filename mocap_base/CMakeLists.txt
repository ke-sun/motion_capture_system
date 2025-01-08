cmake_minimum_required(VERSION 3.8)
project(mocap_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Kalman Filter Library
add_library(mocap_kalman_filter
  src/KalmanFilter.cpp
)

target_link_libraries(mocap_kalman_filter
  ${Boost_LIBRARIES}
  Eigen3::Eigen
  rclcpp
)
ament_target_dependencies(mocap_kalman_filter Eigen3 rclcpp)

# MoCap Driver Library
add_library(mocap_base_driver
  src/MoCapDriverBase.cpp
)

ament_target_dependencies(mocap_base_driver
  rclcpp
  geometry_msgs
  tf2_eigen
  tf2_ros
  nav_msgs
)

target_link_libraries(mocap_base_driver
  mocap_kalman_filter
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS mocap_base_driver mocap_kalman_filter
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()


