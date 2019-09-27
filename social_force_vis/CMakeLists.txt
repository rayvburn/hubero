cmake_minimum_required(VERSION 2.8.3)
project(social_force_vis)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include lib
	CATKIN_DEPENDS gazebo_ros roscpp rospy visualization_msgs
)

find_package(gazebo REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  src
  lib
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#####################
## Gazebo-specific ##
#####################

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++14")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#############
## Library ##
#############

add_library( ${PROJECT_NAME} SHARED
 #include/SFMVisGrid.h
 #src/SFMVisGrid.cpp
 #include/SFMVisPoint.h
 #src/SFMVisPoint.cpp
 include/MarkerBase.h
 include/Arrow.h
 include/LineList.h
 include/Text.h
 include/Grid.h
 src/MarkerBase.cpp
 src/Arrow.cpp
 src/LineList.cpp
 src/Text.cpp
 src/Grid.cpp
)

add_dependencies( ${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries( ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  ${roscpp_LIBRARIES} 
)

set_target_properties( ${PROJECT_NAME} 
  PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY 
  ${CMAKE_SOURCE_DIR}/lib
)

# messing around with below
target_include_directories( ${PROJECT_NAME} 
  PRIVATE 
  include
  src
  lib
)