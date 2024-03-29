cmake_minimum_required(VERSION 3.5)
project(hubero_core
	LANGUAGES CXX
)

add_definitions(-std=c++14)

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

set(ACTOR_LIB_NAME hubero_actor)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   # libs/fsmlite/src: workaround for 'fsm.h' not being found when another package tries to use 'hubero_core' classes
   INCLUDE_DIRS include libs/fsmlite/src
   LIBRARIES ${ACTOR_LIB_NAME}
   DEPENDS hubero_common hubero_interfaces
)

###########
## Build ##
###########
# NOTE: header-only files (e.g., FSM and task brief definitions) are not listed here
add_library(${ACTOR_LIB_NAME} SHARED
   src/actor.cpp
   src/fsm_super.cpp
)
target_link_libraries(${ACTOR_LIB_NAME}
   ${hubero_common_LIBRARIES}
   ${hubero_interfaces_LIBRARIES}
   fsmlite::fsmlite
)

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

##########
## Test ##
##########
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_fsm_super test/test_fsm_super.cpp)
  target_link_libraries(test_fsm_super ${ACTOR_LIB_NAME})

  catkin_add_gtest(test_fsm_tasks test/test_fsm_tasks.cpp)
  target_link_libraries(test_fsm_tasks ${ACTOR_LIB_NAME})

  catkin_add_gtest(test_task_predicates test/test_task_predicates.cpp)
  target_link_libraries(test_task_predicates ${ACTOR_LIB_NAME})

  catkin_add_gtest(test_task_status test/test_task_status.cpp)
  target_link_libraries(test_task_status ${ACTOR_LIB_NAME})

  catkin_add_gtest(test_task_request_objectives test/test_task_request_objectives.cpp)
  target_link_libraries(test_task_request_objectives ${ACTOR_LIB_NAME})

  catkin_add_gtest(test_task_request_arg_validity test/test_task_request_arg_validity.cpp)
  target_link_libraries(test_task_request_arg_validity ${ACTOR_LIB_NAME})

  catkin_add_gtest(test_navigation_base test/test_navigation_base.cpp)
  target_link_libraries(test_navigation_base ${ACTOR_LIB_NAME})

  catkin_add_gtest(test_navigation_predicates test/test_navigation_predicates.cpp)
  target_link_libraries(test_navigation_predicates ${ACTOR_LIB_NAME})
endif()
