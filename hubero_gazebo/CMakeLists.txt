cmake_minimum_required(VERSION 3.5)
project(hubero_gazebo)

add_definitions(-std=c++14)

# prevent some unrelated warnings
cmake_policy(SET CMP0054 OLD)
include(FindPackageHandleStandardArgs)
set(FPHSA_NAME_MISMATCHED TRUE)

find_package(catkin REQUIRED
    COMPONENTS
        hubero_core
        hubero_ros
)
find_package(gazebo REQUIRED)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${hubero_core_INCLUDE_DIRS}
    ${hubero_ros_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set(HUBERO_INTERFACE_LIB_NAME ${PROJECT_NAME}_interface)
set(ACTOR_PLUGIN_LIB_NAME ${PROJECT_NAME}_actor)

# CATKIN PACKAGE
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${HUBERO_INTERFACE_LIB_NAME} ${ACTOR_PLUGIN_LIB_NAME}
    CATKIN_DEPENDS hubero_core hubero_ros
    DEPENDS GAZEBO
)

# BUILD
add_library(${HUBERO_INTERFACE_LIB_NAME} SHARED
    include/${PROJECT_NAME}/animation_control_gazebo.h
    include/${PROJECT_NAME}/localisation_gazebo.h
    include/${PROJECT_NAME}/model_control_gazebo.h
    include/${PROJECT_NAME}/time_gazebo.h
    include/${PROJECT_NAME}/world_geometry_gazebo.h
    src/animation_control_gazebo.cpp
    src/localisation_gazebo.cpp
    src/model_control_gazebo.cpp
    src/time_gazebo.cpp
    src/world_geometry_gazebo.cpp
)
target_link_libraries(${HUBERO_INTERFACE_LIB_NAME}
    ${hubero_core_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

add_library(${ACTOR_PLUGIN_LIB_NAME} SHARED
  include/${PROJECT_NAME}/actor_plugin_gazebo.h
  src/actor_plugin_gazebo.cpp
)
target_link_libraries(${ACTOR_PLUGIN_LIB_NAME}
    ${HUBERO_INTERFACE_LIB_NAME}
    ${GAZEBO_LIBRARIES}
    ${hubero_ros_LIBRARIES}
)

# INSTALL
install(TARGETS ${HUBERO_INTERFACE_LIB_NAME} ${ACTOR_PLUGIN_LIB_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# TEST
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_localisation test/test_localisation.cpp)
  target_link_libraries(test_localisation ${HUBERO_INTERFACE_LIB_NAME})
endif()
