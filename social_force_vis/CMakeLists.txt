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
	INCLUDE_DIRS include
	CATKIN_DEPENDS gazebo_ros roscpp rospy visualization_msgs
)

find_package(gazebo REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  src
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++14")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(sfm_vis
 include/SFMVis.h
 src/SFMVis.cpp
)

add_dependencies(sfm_vis ${catkin_EXPORTED_TARGETS})
target_link_libraries(sfm_vis ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
