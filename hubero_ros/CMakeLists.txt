cmake_minimum_required(VERSION 2.8.3)
project(hubero_ros)

add_definitions(-std=c++14)

find_package(catkin REQUIRED
	COMPONENTS
		hubero_common
		hubero_interfaces
		cmake_modules
		nav_core
		move_base
		nav_msgs
		pluginlib
		roscpp
		tf
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${hubero_common_INCLUDE_DIRS}
	${hubero_interfaces_INCLUDE_DIRS}
)

set(HUBERO_NAV_ROS hubero_navigation_ros)
set(HUBERO_TASK_ROS hubero_task_request_ros)

catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		${HUBERO_NAV_ROS}
		${HUBERO_TASK_ROS}
	CATKIN_DEPENDS
		hubero_common
		hubero_interfaces
		nav_msgs
		pluginlib
		roscpp
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
	${catkin_LIBRARIES}
)

## Install
install(TARGETS ${HUBERO_NAV_ROS} ${HUBERO_TASK_ROS}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES nav_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)
