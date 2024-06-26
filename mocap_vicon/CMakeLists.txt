cmake_minimum_required(VERSION 3.8)
project(mocap_vicon)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap_base REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread chrono)

find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rcutils REQUIRED)

# dependecies
set(dependencies
    rclcpp
    std_msgs
    mocap_base
    Eigen3
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    geometry_msgs
    Boost)


include_directories(include)
add_subdirectory(src/vicon_sdk)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# link subdirectory with header file

add_library(mocap_vicon_driver src/ViconDriver.cpp)
target_link_libraries(mocap_vicon_driver ViconDataStreamSDK_CPP)
ament_target_dependencies(mocap_vicon_driver ${dependencies})

# executable and its supoort files

add_executable(mocap_vicon_node src/vicon.cpp)
target_link_libraries(mocap_vicon_node mocap_vicon_driver)
ament_target_dependencies(mocap_vicon_node ${dependencies})


# to find executable during install
install(TARGETS
        mocap_vicon_node
        DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
