cmake_minimum_required(VERSION 2.8.3)
project(actor_global_plan)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	roscpp
	tf
	geometry_msgs
	costmap_2d
	nav_core
	navfn
	global_planner
)

catkin_package(
	CATKIN_DEPENDS 
	gazebo_ros
	roscpp 
	tf
	geometry_msgs
	costmap_2d
	nav_core
	navfn
	global_planner
)

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++14")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

