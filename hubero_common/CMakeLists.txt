cmake_minimum_required(VERSION 2.8.3)
project(hubero_common
	LANGUAGES CXX
)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(IGN_MATH_VER 3)

find_package(catkin REQUIRED COMPONENTS)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)

include_directories(include ${IGNITION-MATH_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   DEPENDS IGNITION-MATH
)

###########
## Build ##
###########
add_library(${PROJECT_NAME} SHARED
   include/hubero_common/defines.h
   include/hubero_common/logger.h
   include/hubero_common/time.h
   include/hubero_common/typedefs.h
)
target_link_libraries(${PROJECT_NAME} ${IGNITION-MATH_LIBRARIES})
# LINKER_LANGUAGE explicitly defined since this is header-only library (no cpp file)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
