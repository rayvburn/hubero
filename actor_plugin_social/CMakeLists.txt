cmake_minimum_required(VERSION 2.8.3)
project(actor_plugin_social)

# note: fuzzylite library utilized in `fuzz` namespace
find_library(fuzzylite_LIBRARY fuzzylite $ENV{HOME}/libraries/fuzzylite/fuzzylite/release/bin) # path to the .so file
include_directories($ENV{HOME}/libraries/fuzzylite/fuzzylite) # include fuzzylite's headers folder

find_package(roscpp REQUIRED)
find_package(actor_sim_utils REQUIRED)
find_package(actor_sim_srv REQUIRED)
find_package(actor_sim_action REQUIRED)
find_package(actor_global_plan REQUIRED)
find_package(catkin REQUIRED COMPONENTS visualization_msgs)

find_package(tf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${actor_sim_utils_INCLUDE_DIRS})
include_directories(${actor_sim_srv_INCLUDE_DIRS})
include_directories(${actor_sim_action_INCLUDE_DIRS})
include_directories(${actor_global_plan_INCLUDE_DIRS})

include_directories(${tf_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${std_srvs_INCLUDE_DIRS})

include_directories(/user/include/sdformat-5.2)
include_directories(./include)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


catkin_package(CATKIN_DEPENDS roscpp)


add_library(${PROJECT_NAME} SHARED 
  include/ActorPluginSocial.h 
  src/ActorPluginSocial.cpp
)

# CommonInfo doesnt see Ellipse - namespace?
# solution: move headers into include/, sources into src/
add_library(actor_core

  include/actor/core/Actor.h 
  include/actor/core/CommonInfo.h 
  include/actor/core/FSM.h 
  include/actor/core/Target.h
  include/actor/core/TargetLot.h
  include/actor/core/LieDownHelper.h
  include/actor/core/Path.h
  include/actor/core/Voice.h
  include/actor/core/Action.h
  include/actor/core/Velocity.h
  include/actor/core/FollowObjectHelper.h
  include/actor/core/StanceHelper.h

  src/actor/core/Actor.cpp 
  src/actor/core/CommonInfo.cpp 
  src/actor/core/FSM.cpp
  src/actor/core/Target.cpp
  src/actor/core/LieDownHelper.cpp
  src/actor/core/Path.cpp
  src/actor/core/Voice.cpp
  src/actor/core/Action.cpp
  src/actor/core/Velocity.cpp
  src/actor/core/FollowObjectHelper.cpp
  src/actor/core/StanceHelper.cpp

)

add_library(actor_frame
  include/actor/FrameGlobal.h
  src/actor/FrameGlobal.cpp
)

add_library(actor_inflation
  
  include/actor/inflation/Border.h
  include/actor/inflation/Box.h
  include/actor/inflation/Circle.h
  include/actor/inflation/Ellipse.h

  src/actor/inflation/Border.cpp
  src/actor/inflation/Box.cpp
  src/actor/inflation/Circle.cpp
  src/actor/inflation/Ellipse.cpp

)

add_library(actor_ros 

  include/actor/ros_interface/Connection.h
  include/actor/ros_interface/Stream.h
  include/actor/ros_interface/Node.h
  include/actor/ros_interface/ParamLoader.h
  include/actor/ros_interface/Conversion.h
  include/actor/ros_interface/GlobalPlan.h

  src/actor/ros_interface/Connection.cpp
  src/actor/ros_interface/Stream.cpp
  src/actor/ros_interface/Node.cpp
  src/actor/ros_interface/ParamLoader.cpp
  src/actor/ros_interface/Conversion.cpp
  src/actor/ros_interface/GlobalPlan.cpp

)

add_library(fuzzy

  include/fuzz/Processor.h
  include/fuzz/TrapezoidParted.h
  include/fuzz/TrapezoidLocIndep.h
  include/fuzz/TrapezoidLocDep.h
  include/fuzz/SocialBehavioursDb.h
  include/fuzz/SocialConductor.h

  src/fuzz/Processor.cpp
  src/fuzz/TrapezoidParted.cpp
  src/fuzz/TrapezoidLocIndep.cpp
  src/fuzz/TrapezoidLocDep.cpp
  src/fuzz/SocialBehavioursDb.cpp
  src/fuzz/SocialConductor.cpp

)

add_library(sfm_core

  # SFM core
  include/sfm/core/SocialForceModel.h 
  include/sfm/core/Inflator.h 
  include/sfm/core/SFMDebug.h
  include/sfm/core/ActorInfoDecoder.h
  include/sfm/core/ShiftRegister.h
  include/sfm/core/WorldBoundary.h

  src/sfm/core/SocialForceModel.cpp
  src/sfm/core/Inflator.cpp
  src/sfm/core/SFMDebug.cpp
  src/sfm/core/ActorInfoDecoder.cpp
  src/sfm/core/WorldBoundary.cpp

)
 
add_library(sfm_vis

 # Visualization
 include/sfm/vis/MarkerBase.h
 include/sfm/vis/Arrow.h
 include/sfm/vis/LineList.h
 include/sfm/vis/Text.h
 include/sfm/vis/HeatCell.h
 include/sfm/vis/Grid.h
 include/sfm/vis/GridForce.h
 include/sfm/vis/Heatmap.h

 src/sfm/vis/MarkerBase.cpp
 src/sfm/vis/Arrow.cpp
 src/sfm/vis/LineList.cpp
 src/sfm/vis/Text.cpp
 src/sfm/vis/HeatCell.cpp
 src/sfm/vis/Grid.cpp
 src/sfm/vis/GridForce.cpp
 src/sfm/vis/Heatmap.cpp

)

# link the external library
target_link_libraries(fuzzy ${fuzzylite_LIBRARY})

# link the library from another package
target_link_libraries(actor_ros 

	# catkin_LIBRARIES - if the library depends on libraries provided by other catkin package(s)
	# ${catkin_LIBRARIES}

  # link common library
  actor_frame
	
)
target_link_libraries(actor_inflation actor_frame)
target_link_libraries(actor_ros actor_frame)


#add_library(enums include/Enums.h) # (header only) -> DEPRECATED

# libraries linked to `ActorPluginSocial`
target_link_libraries( ${PROJECT_NAME} 

  actor_core
  actor_inflation
  actor_ros
  fuzzy

  # enums -> DEPRECATED

  sfm_core
  sfm_vis

  ${GAZEBO_LIBRARIES}  
  ${roscpp_LIBRARIES} 
  ${tf_LIBRARIES}
  ${catkin_LIBRARIES}

)

set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
)