cmake_minimum_required(VERSION 2.8.3)
project(hubero_interface)

# based on: https://github.com/ros-planning/navigation/blob/kinetic-devel/nav_core/CMakeLists.txt

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hubero_common
  roscpp
  nav_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)

catkin_package(
    INCLUDE_DIRS
      include
    CATKIN_DEPENDS
      hubero_common
      roscpp
      nav_msgs
      std_msgs
      sensor_msgs
      geometry_msgs
      tf
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)