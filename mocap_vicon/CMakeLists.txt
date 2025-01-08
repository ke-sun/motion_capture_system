cmake_minimum_required(VERSION 3.5)
project(mocap_vicon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap_base REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(Eigen3 REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   geometry_msgs
#   tf
#   tf_conversions
#   mocap_base
# )

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

# Include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mocap_base_INCLUDE_DIRS}
)


add_subdirectory(src/vicon_sdk)

add_library(mocap_vicon_driver
  src/ViconDriver.cpp
)

ament_target_dependencies(mocap_vicon_driver mocap_base Eigen3 rclcpp tf2_eigen mocap_base geometry_msgs nav_msgs)

target_link_libraries(mocap_vicon_driver
  ViconDataStreamSDK_CPP
  ${VICON_BOOST_LIBS}
  Eigen3::Eigen
)


add_executable(mocap_vicon_node
  src/vicon.cpp
)

ament_target_dependencies(mocap_vicon_node Eigen3 rclcpp tf2_eigen  geometry_msgs nav_msgs)

target_link_libraries(mocap_vicon_node
  mocap_vicon_driver
  ViconDataStreamSDK_CPP
  ${VICON_BOOST_LIBS}
  Eigen3::Eigen
)


install(TARGETS mocap_vicon_node mocap_vicon_driver 
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
