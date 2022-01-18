cmake_minimum_required(VERSION 2.8.3)
project(hubero_ros)

add_definitions(-std=c++14)

find_package(catkin REQUIRED
	COMPONENTS
		hubero_common
		hubero_interfaces
		hubero_ros_msgs
		cmake_modules
		nav_core
		move_base
		nav_msgs
		roscpp
		actionlib
		tf
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${hubero_common_INCLUDE_DIRS}
	${hubero_interfaces_INCLUDE_DIRS}
	${hubero_ros_msgs_INCLUDE_DIRS}
)

set(HUBERO_NAV_ROS hubero_navigation_ros)
set(HUBERO_TASK_ROS hubero_task_request_ros)
set(HUBERO_TASK_ROS_API hubero_task_request_ros_api)
set(HUBERO_NODE_ROS hubero_node_ros)

catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		${HUBERO_NAV_ROS}
		${HUBERO_TASK_ROS}
		${HUBERO_TASK_ROS_API}
		${HUBERO_NODE_ROS}
	CATKIN_DEPENDS
		hubero_common
		hubero_interfaces
		hubero_ros_msgs
		nav_msgs
		roscpp
		actionlib
)

## Node library for HuBeRo
add_library(${HUBERO_NODE_ROS}
	include/${PROJECT_NAME}/node.h
	src/node.cpp
)
add_dependencies(${HUBERO_NODE_ROS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${HUBERO_NODE_ROS}
	${catkin_LIBRARIES}
)

## Navigation library for HuBeRo
add_library(${HUBERO_NAV_ROS}
	include/${PROJECT_NAME}/navigation_ros.h
	include/${PROJECT_NAME}/utils/converter.h
	src/navigation_ros.cpp
	src/utils/converter.cpp
)
add_dependencies(${HUBERO_NAV_ROS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${HUBERO_NAV_ROS}
	${hubero_interfaces_LIBRARIES}
	${hubero_common_LIBRARIES}
	${HUBERO_NODE_ROS}
	${catkin_LIBRARIES}
)

## Task requesting library for HuBeRo
add_library(${HUBERO_TASK_ROS}
	include/${PROJECT_NAME}/task_request_ros.h
	src/task_request_ros.cpp
)
target_link_libraries(${HUBERO_TASK_ROS}
	${hubero_interfaces_LIBRARIES}
	${hubero_common_LIBRARIES}
	${HUBERO_NODE_ROS}
	${catkin_LIBRARIES}
)

## Task requesting client library for HuBeRo
add_library(${HUBERO_TASK_ROS_API}
	include/${PROJECT_NAME}/task_request_ros_api.h
	src/task_request_ros_api.cpp
)
target_link_libraries(${HUBERO_TASK_ROS_API}
	${hubero_interfaces_LIBRARIES}
	${hubero_common_LIBRARIES}
	${HUBERO_NODE_ROS}
	${catkin_LIBRARIES}
)

## Install
install(TARGETS ${HUBERO_NAV_ROS} ${HUBERO_TASK_ROS} ${HUBERO_TASK_ROS_API} ${HUBERO_NODE_ROS}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)

## Test
if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(test_task_request test/test_task_request.cpp)
	target_link_libraries(test_task_request ${HUBERO_TASK_ROS})

	catkin_add_gtest(test_navigation test/test_navigation.cpp)
	target_link_libraries(test_navigation ${HUBERO_NAV_ROS})
endif()
