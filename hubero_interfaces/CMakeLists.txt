cmake_minimum_required(VERSION 3.5)
project(hubero_interfaces)

# based on: https://github.com/ros-planning/navigation/blob/kinetic-devel/nav_core/CMakeLists.txt

add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hubero_common
)

catkin_package(
    INCLUDE_DIRS
      include
    CATKIN_DEPENDS
      hubero_common
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
