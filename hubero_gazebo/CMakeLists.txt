cmake_minimum_required(VERSION 2.8.3)
project(hubero_ros)

find_package(catkin REQUIRED
        COMPONENTS
            hubero_common
            hubero_interface
            cmake_modules
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            roscpp
            tf
        )
set(IGN_MATH_VER 3)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)

# find_package(Eigen3 REQUIRED)
# find_package(PCL REQUIRED)
# remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${hubero_common_INCLUDE_DIRS}
    ${hubero_interface_INCLUDE_DIRS}
    ${IGNITION-MATH_INCLUDE_DIRS}
    # ${EIGEN3_INCLUDE_DIRS}
    # ${PCL_INCLUDE_DIRS}
    )
# add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#     cfg/DWAPlanner.cfg
# )

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        hubero_localisation_gazebo
    CATKIN_DEPENDS
        hubero_common
        hubero_interface
        dynamic_reconfigure
        nav_msgs
        pluginlib
        roscpp
)

add_library(hubero_localisation_gazebo src/navigation_ros.cpp)
add_dependencies(hubero_localisation_gazebo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hubero_localisation_gazebo ${hubero_common_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS hubero_localisation_gazebo
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES hubero_gazebo_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# set_target_properties(hubero_localisation_gazebo PROPERTIES
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
# )