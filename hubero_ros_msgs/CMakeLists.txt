cmake_minimum_required(VERSION 2.8.3)
project(hubero_ros_msgs)

add_definitions(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
    action
  FILES
    FollowObject.action
    LieDown.action
    LieDownObject.action
    # MoveAround.action
    MoveToGoal.action
    MoveToObject.action
    # Run.action
    # SitDown.action
    # SitDownObject.action
    # Stand.action
    # Talk.action
    # Teleop.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
   actionlib_msgs
)

## catkin specific configuration
catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_generation
)