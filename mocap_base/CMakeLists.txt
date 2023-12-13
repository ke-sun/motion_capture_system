cmake_minimum_required(VERSION 3.8)
project(mocap_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(rcutils REQUIRED)

set(dependencies
    rclcpp
    rcutils
    nav_msgs
    tf2
    tf2_eigen
    geometry_msgs
    Eigen3
    Boost
    )

include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# setup targets
add_library(${PROJECT_NAME} SHARED
            src/MoCapDriverBase.cpp
            src/KalmanFilter.cpp)          
target_link_libraries(${PROJECT_NAME} Boost::thread)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(DIRECTORY include/
        DESTINATION include/)


#to find executable during install
install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
