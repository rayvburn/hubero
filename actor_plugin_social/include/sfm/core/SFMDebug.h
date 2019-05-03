/*
 * SFMDebug.h
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_CORE_SFMDEBUG_H_
#define INCLUDE_SFM_CORE_SFMDEBUG_H_

#pragma once // same as above..

#define SILENT_

// deprecated
//#define SFM_RIGHT_SIDE 0
//#define SFM_LEFT_SIDE  1
//#define SFM_BEHIND     2

// #define THETA_ALPHA_BETA_CONSIDER_ZERO_VELOCITY
// #define BETA_REL_LOCATION_BASED_ON_NORMAL


// #define DEBUG_SFM_PARAMETERS

#ifndef SILENT_

//#define DEBUG_GEOMETRY_1 // angle correctes etc.
//#define DEBUG_GEOMETRY_2 // relative location
//#define DEBUG_INTERNAL_ACC
//#define DEBUG_INTERACTION_FORCE
//#define DEBUG_REL_SPEED
//#define DEBUG_NEW_POSE
//#define DEBUG_ACTOR_FACING_TARGET
//#define DEBUG_BOUNDING_BOX
//#define DEBUG_BOUNDING_CIRCLE // each iteration
//#define DEBUG_CLOSEST_POINTS
//#define DEBUG_OSCILLATIONS	// each iteration (table1 & actor1)
//#define DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX 			// GetClosestPointsOfIntersectedModels()


#endif

// each iteration but only when print_data is true!



#define DEBUG_LOG_ALL_INTERACTIONS
//
// #define DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX_BB				// old
// #define DEBUG_BOUNDING_ELLIPSE_INTERSECTION

//#define DEBUG_FORCE_EACH_OBJECT 									// detailed info in each iteration
//#define DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE

//#define DEBUG_YAW_MOVEMENT_DIR	// each iteration




//#define DEBUG_SHORT_DISTANCE	// force printing info when distance to an obstacle is small

#ifdef DEBUG_NEW_POSE
#define DEBUG_JUMPING_POSITION
unsigned int curr_actor = 3;				// TODO: couldn't catch an event to debug this
#endif

// ---------------------------------

#define CALCULATE_INTERACTION

// ---------------------------------

#define NEW_YAW_BASE						// DEPRECATED?

// ---------------------------------

// #define DEBUG_FUZZIFIER_VEL_ANGLE
// #define DEBUG_FUZZIFIER_CONDITION_INFO

// ---------------------------------

#include <string>

// variables for debugging
static std::string debug_current_actor_name;
static std::string debug_current_object_name;
static bool print_data_dbg = false;

void SfmDebugSetCurrentActorName(const std::string &name);
void SfmDebugSetCurrentObjectName(const std::string &name);
void SfmSetPrintData(const bool &to_print);

std::string SfmDebugGetCurrentActorName();
std::string SfmDebugGetCurrentObjectName();
bool SfmGetPrintData();

// ---------------------------------


#endif /* INCLUDE_SFM_CORE_SFMDEBUG_H_ */
