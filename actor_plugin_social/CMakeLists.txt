cmake_minimum_required(VERSION 2.8.3)
project(actor_plugin_social)

# note: fuzzylite library utilized in `fuzz` namespace
find_library(fuzzylite_LIBRARY fuzzylite $ENV{HOME}/libraries/fuzzylite/fuzzylite/release/bin) # path to the .so file
include_directories($ENV{HOME}/libraries/fuzzylite/fuzzylite) # include fuzzylite's headers folder

find_package(roscpp REQUIRED)
find_package(actor_sim_utils REQUIRED)
find_package(actor_sim_srv REQUIRED)
find_package(incare_human_robot_communication REQUIRED)
find_package(actor_global_plan REQUIRED)
find_package(catkin REQUIRED COMPONENTS visualization_msgs)

find_package(tf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${actor_sim_utils_INCLUDE_DIRS})
include_directories(${actor_sim_srv_INCLUDE_DIRS})
include_directories(${incare_human_robot_communication_INCLUDE_DIRS})
include_directories(${actor_global_plan_INCLUDE_DIRS})

include_directories(${tf_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${std_srvs_INCLUDE_DIRS})

include_directories(/user/include/sdformat-5.2)
include_directories(./include)

link_directories(${incare_human_robot_communication_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

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
  include/actor/core/LieDownHelper.h
  include/actor/core/Path.h

  src/actor/core/Actor.cpp 
  src/actor/core/CommonInfo.cpp 
  src/actor/core/FSM.cpp
  src/actor/core/Target.cpp
  src/actor/core/LieDownHelper.cpp
  src/actor/core/Path.cpp

)

add_library(actor_inflation
  
  include/actor/inflation/Box.h
  include/actor/inflation/Circle.h
  include/actor/inflation/Ellipse.h

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

  src/sfm/core/SocialForceModel.cpp
  src/sfm/core/Inflator.cpp
  src/sfm/core/SFMDebug.cpp
  src/sfm/core/ActorInfoDecoder.cpp
)
 
add_library(sfm_vis

 # Visualization
 include/sfm/vis/MarkerBase.h
 include/sfm/vis/Arrow.h
 include/sfm/vis/LineList.h
 include/sfm/vis/Text.h
 include/sfm/vis/Grid.h

 src/sfm/vis/MarkerBase.cpp
 src/sfm/vis/Arrow.cpp
 src/sfm/vis/LineList.cpp
 src/sfm/vis/Text.cpp
 src/sfm/vis/Grid.cpp

)

# link the external library
target_link_libraries(fuzzy ${fuzzylite_LIBRARY})

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
  ${incare_human_robot_communication_LIBRARIES} # in case of a bad inclusion something like this should help - /home/rayvburn/ros_workspace/ws_people_sim/src/incare_human_robot_communication/lib/libincare_human_robot_communication.so

)

set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
)