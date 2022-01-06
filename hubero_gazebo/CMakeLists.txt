cmake_minimum_required(VERSION 2.8.3)
project(hubero_gazebo)

find_package(catkin REQUIRED
    COMPONENTS
        hubero_common
        hubero_interfaces
        # roscpp
)
find_package(gazebo REQUIRED)

include_directories(
    include
    # ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${hubero_common_INCLUDE_DIRS}
    ${hubero_interfaces_INCLUDE_DIRS}
)
# TODO: ${GAZEBO_INCLUDE_DIRS} should provide that
# include_directories(/usr/include/sdformat-5.2)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# LIBRARIES
add_library(${PROJECT_NAME}_interface SHARED
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
target_link_libraries(${PROJECT_NAME}_interface
    ${hubero_common_LIBRARIES}
)

add_library(${PROJECT_NAME}_actor SHARED
  include/${PROJECT_NAME}/actor_plugin_gazebo.h
  src/actor_plugin_gazebo.cpp
)
target_link_libraries(${PROJECT_NAME}_actor
    ${PROJECT_NAME}_interface
    ${GAZEBO_LIBRARIES}
)

# CATKIN PACKAGE
catkin_package(
    INCLUDE_DIRS
        include
    # LIBRARIES
    #     hubero_localisation_gazebo
    CATKIN_DEPENDS
        hubero_common
        hubero_interfaces
        # roscpp
)

# INSTALL
install(TARGETS ${PROJECT_NAME}_actor
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

set_target_properties(${PROJECT_NAME}_actor PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
)
