cmake_minimum_required(VERSION 2.8.3)
project(actor_plugin_social)

find_package(roscpp REQUIRED)
find_package(actor_sim_utils REQUIRED)
find_package(actor_sim_srv REQUIRED)
#find_package(social_force_vis REQUIRED)
find_package(catkin REQUIRED COMPONENTS social_force_vis)

find_package(tf REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(grid_map_ros)

#find_library(
#  PATHS /home/rayvburn/ros_workspace/ws_people_sim/src/incare_people_sim/social_force_vis/lib
#)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${actor_sim_utils_INCLUDE_DIRS})
include_directories(${actor_sim_srv_INCLUDE_DIRS})
include_directories(${social_force_vis_INCLUDE_DIRS})

include_directories(${tf_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
# include_directories(${grid_map_ros_INCLUDE_DIRS})
include_directories(/user/include/sdformat-5.2)
include_directories(./include)

link_directories(${social_force_vis_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED include/ActorPluginSocial.h src/ActorPluginSocial.cpp)
add_library(social_force_model include/sfm/core/SocialForceModel.h src/sfm/core/SocialForceModel.cpp)
add_library(social_force_model_utils include/SocialForceModelUtils.h src/SocialForceModelUtils.cpp)
#add_library(actor_bounding_circle include/BoundingCircle.h src/BoundingCircle.cpp)
#add_library(actor_common_info include/CommonInfo.h src/CommonInfo.cpp)

# CommonInfo doesnt see Ellipse - namespace?
# sol: headers into include, sources into src
add_library(actor_inflation
  include/inflation/Box.h
  src/inflation/Box.cpp
  include/inflation/Circle.h
  src/inflation/Circle.cpp
  include/inflation/Ellipse.h
  src/inflation/Ellipse.cpp
)

add_library(actor_utils 

  include/BoundingEllipse.h 
  include/BoundingCircle.h 
  include/CommonInfo.h 
  src/BoundingEllipse.cpp
  src/BoundingCircle.cpp 
  src/CommonInfo.cpp 
  include/ActorROSInterface.h 
  src/ActorROSInterface.cpp

)



target_link_libraries( actor_inflation actor_utils)

#add_library(enums include/Enums.h)

target_link_libraries( ${PROJECT_NAME} 
  social_force_model 
  social_force_model_utils
  #actor_bounding_circle
  #actor_common_info
  actor_inflation
  actor_utils
  
  #enums
  ${GAZEBO_LIBRARIES}  
  ${roscpp_LIBRARIES} 
  ${tf_LIBRARIES}
  ${catkin_LIBRARIES}
#  ${social_force_vis_LIBRARIES} # catkin sees it, gazebo seems to not - plugin does not load
  /home/rayvburn/ros_workspace/ws_people_sim/src/gazebo_ros_people_sim/social_force_vis/lib/libsocial_force_vis.so 
#  social_force_vis
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

