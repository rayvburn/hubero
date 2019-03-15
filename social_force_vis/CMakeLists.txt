cmake_minimum_required(VERSION 2.8.3)
project(social_force_vis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy visualization_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

#add_library(sfm_vis
# src/SFMVis.h
# src/SFMVis.cpp
#)
