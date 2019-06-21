cmake_minimum_required(VERSION 2.8.3)
project(actor_plugin_social)

find_package(roscpp REQUIRED)
find_package(actor_sim_utils REQUIRED)
find_package(actor_sim_srv REQUIRED)
find_package(catkin REQUIRED COMPONENTS social_force_vis)

find_package(tf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) #################
find_package(costmap_2d REQUIRED) ####################
find_package(nav_core REQUIRED) ######################
find_package(navfn REQUIRED) #########################

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${actor_sim_utils_INCLUDE_DIRS})
include_directories(${actor_sim_srv_INCLUDE_DIRS})
include_directories(${social_force_vis_INCLUDE_DIRS})

include_directories(${tf_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS}) ###
include_directories(${costmap_2d_INCLUDE_DIRS}) ######
include_directories(${nav_core_INCLUDE_DIRS}) ########
include_directories(${navfn_INCLUDE_DIRS}) ###########

include_directories(/user/include/sdformat-5.2)
include_directories(./include)

link_directories(${social_force_vis_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED include/ActorPluginSocial.h src/ActorPluginSocial.cpp)

add_library(social_force_model 
  include/sfm/core/SocialForceModel.h 
  include/sfm/core/Inflator.h 
  include/sfm/core/SFMDebug.h
  include/sfm/core/ActorInfoDecoder.h

  src/sfm/core/SocialForceModel.cpp
  src/sfm/core/Inflator.cpp
  src/sfm/core/SFMDebug.cpp
  src/sfm/core/ActorInfoDecoder.cpp
)

add_library(social_force_model_fuzzy
  include/sfm/fuzz/Fuzzifier.h
  include/sfm/fuzz/Defuzzifier.h
  src/sfm/fuzz/Fuzzifier.cpp
  src/sfm/fuzz/Defuzzifier.cpp)

add_library(social_force_model_utils include/SocialForceModelUtils.h src/SocialForceModelUtils.cpp)

# CommonInfo doesnt see Ellipse - namespace?
# solution: move headers into include/, sources into src/
add_library(actor_inflation
  include/inflation/Box.h
  src/inflation/Box.cpp
  include/inflation/Circle.h
  src/inflation/Circle.cpp
  include/inflation/Ellipse.h
  src/inflation/Ellipse.cpp
)

add_library(actor_core 
  include/core/Actor.h 
  include/core/CommonInfo.h 
  include/core/FSM.h 
  src/core/Actor.cpp 
  src/core/CommonInfo.cpp 
  src/core/FSM.cpp
)

add_library(actor_ros 
  #include/ActorROSInterface.h 
  #src/ActorROSInterface.cpp

  include/ros_interface/Connection.h
  src/ros_interface/Connection.cpp
  include/ros_interface/Stream.h
  src/ros_interface/Stream.cpp
  include/ros_interface/Node.h
  src/ros_interface/Node.cpp

  include/ros_interface/ParamLoader.h
  src/ros_interface/ParamLoader.cpp

  include/ros_interface/GlobalPlan.h
  src/ros_interface/GlobalPlan.cpp
)

target_link_libraries(actor_inflation actor_core actor_ros) # ros needed here?
target_link_libraries(social_force_model social_force_model_fuzzy)
#target_link_libraries(actor_core actor_ros)

#add_library(enums include/Enums.h)

target_link_libraries( ${PROJECT_NAME} 
  social_force_model 
  social_force_model_utils
  #actor_bounding_circle
  #actor_common_info
  actor_inflation
  actor_core
  actor_ros
  #enums
  ${GAZEBO_LIBRARIES}  
  ${roscpp_LIBRARIES} 
  ${tf_LIBRARIES}
  ${catkin_LIBRARIES}
  ${costmap_2d_LIBRARIES}
  ${nav_core_LIBRARIES}
  ${navfn_LIBRARIES}
#  ${social_force_vis_LIBRARIES} # catkin sees it, gazebo seems to not - plugin does not load
  /home/rayvburn/ros_workspace/ws_people_sim/src/gazebo_ros_people_sim/social_force_vis/lib/libsocial_force_vis.so 
#  social_force_vis
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)