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

	# service section
	std_msgs
	geometry_msgs
	message_generation
)


## service section ##

add_service_files(
  FILES
  MakeNavPlanFrame.srv
  GetCost.srv
  SetTolerance.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

#####################


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

	# service section
	message_runtime
)


## Specify additional locations of header files (useful when .h files are located deeper inside `include` folder)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(costmap2d_multi_frame SHARED 
	include/actor_global_plan/Costmap2dMultiFrame.h 
	src/actor_global_plan/Costmap2dMultiFrame.cpp
)

add_library(global_planner_multi_frame SHARED 
	include/actor_global_plan/GlobalPlannerMultiFrame.h 
	src/actor_global_plan/GlobalPlannerMultiFrame.cpp
)

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++14")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME} global_planner_multi_frame costmap2d_multi_frame ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

