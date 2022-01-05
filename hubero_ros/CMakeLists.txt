cmake_minimum_required(VERSION 2.8.3)
project(hubero_ros)

# Based on https://github.com/ros-planning/navigation/blob/kinetic-devel/base_local_planner/CMakeLists.txt

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

catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		hubero_navigation_ros
	CATKIN_DEPENDS
		hubero_common
		hubero_interfaces
		nav_msgs
		pluginlib
		roscpp
)

add_library(hubero_navigation_ros
	src/navigation_ros.cpp
	src/utils/converter.cpp
)
add_dependencies(hubero_navigation_ros
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
	hubero_navigation_ros
	${hubero_common_LIBRARIES}
	${catkin_LIBRARIES}
)

install(TARGETS hubero_navigation_ros
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nav_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)

# set_target_properties(hubero_navigation_ros PROPERTIES
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
# )
