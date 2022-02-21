cmake_minimum_required(VERSION 3.5)
project(hubero_ros_scenarios)

add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hubero_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${hubero_ros_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    hubero_ros
)

## executables
add_executable(living_room_node src/living_room_node.cpp)
target_link_libraries(living_room_node
	${hubero_ros_LIBRARIES}
	${roscpp_LIBRARIES}
	${catkin_LIBRARIES}
)

add_executable(parking_node src/parking_node.cpp)
target_link_libraries(parking_node
	${hubero_ros_LIBRARIES}
	${roscpp_LIBRARIES}
	${catkin_LIBRARIES}
)
