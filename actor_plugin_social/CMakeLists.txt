cmake_minimum_required(VERSION 2.8.3)
project(actor_plugin_social)

find_package(roscpp REQUIRED)
find_package(actor_sim_utils REQUIRED)
find_package(actor_sim_srv REQUIRED)
find_package(social_force_vis REQUIRED)
find_package(tf REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(grid_map_ros)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${actor_sim_utils_INCLUDE_DIRS})
include_directories(${actor_sim_srv_INCLUDE_DIRS})
include_directories(${social_force_vis_INCLUDE_DIRS})

include_directories(${tf_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
# include_directories(${grid_map_ros_INCLUDE_DIRS})
include_directories(/user/include/sdformat-5.2)
include_directories(./include)

#link_directories(${social_force_vis_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(actor_plugin_social SHARED include/ActorPluginSocial.h src/ActorPluginSocial.cpp)
add_library(social_force_model include/SocialForceModel.h src/SocialForceModel.cpp)
add_library(social_force_model_utils include/SocialForceModelUtils.h src/SocialForceModelUtils.cpp)

target_link_libraries(actor_plugin_social social_force_model social_force_model_utils ${social_force_vis_LIBRARIES} ${GAZEBO_LIBRARIES}  ${roscpp_LIBRARIES} ${tf_LIBRARIES})
set_target_properties(actor_plugin_social PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

