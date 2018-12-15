cmake_minimum_required(VERSION 2.8.3)
project(actor_fake_imu)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS 
	gazebo_ros
	roscpp 
	sensor_msgs
)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++14")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} src/actor_joints_to_imu.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(actor_joints_to_imu src/actor_joints_to_imu.cpp)
target_link_libraries(actor_joints_to_imu ${catkin_LIBRARIES})

