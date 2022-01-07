cmake_minimum_required(VERSION 2.8.3)
project(hubero_core
	LANGUAGES CXX
)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   hubero_common
   hubero_interfaces
)

add_subdirectory(libs/fsmlite)

include_directories(
   include
   ${hubero_common_INCLUDE_DIRS}
   ${hubero_interfaces_INCLUDE_DIRS}
   libs/fsmlite
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   DEPENDS hubero_common hubero_interfaces
)

###########
## Build ##
###########
set(ACTOR_LIB_NAME hubero_actor)


#############
## Install ##
#############
install(TARGETS ${ACTOR_LIB_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

