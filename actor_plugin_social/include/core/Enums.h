/*
 * Enums.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_ENUMS_H_
#define INCLUDE_ACTOR_ENUMS_H_

namespace actor {

/// \brief Gazebo model ID for ActorPlugin objects
static constexpr unsigned int ACTOR_MODEL_TYPE_ID = 32771;

/// \brief Gazebo model ID for static obstacle objects
static constexpr unsigned int STATIC_OBSTACLE_MODEL_TYPE_ID = 3;

// -------------------------

/// \brief Actor's allowable state which its
/// Finite State Machine recognizes
typedef enum {
	ACTOR_STATE_ALIGN_TARGET = 0,//!< ACTOR_STATE_ALIGN_TARGET rotation to make actor face the target
	ACTOR_STATE_STUCK,           //!< ACTOR_STATE_STUCK
	ACTOR_STATE_MOVE_AROUND,     //!< ACTOR_STATE_MOVE_AROUND periodically choose a random goal and try to reach it
	ACTOR_STATE_STOP_AND_STARE,  //!< ACTOR_STATE_STOP_AND_STARE
	ACTOR_STATE_FOLLOW_OBJECT,   //!< ACTOR_STATE_FOLLOW_OBJECT
	ACTOR_STATE_TELEOPERATION    //!< ACTOR_STATE_TELEOPERATION
} ActorState;

/// \brief Actor's initial state
static const ActorState ACTOR_STATE_INITIAL = ACTOR_STATE_ALIGN_TARGET;

// -------------------------

/// \brief Actor's stances, each of them requires
/// a separate animation (`.dae` file)
typedef enum {
	ACTOR_STANCE_WALK = 0,//!< ACTOR_STANCE_WALK walking
	ACTOR_STANCE_STAND,   //!< ACTOR_STANCE_STAND standing
	ACTOR_STANCE_LIE,     //!< ACTOR_STANCE_LIE lying down
	ACTOR_STANCE_SIT_DOWN,//!< ACTOR_STANCE_SIT_DOWN sit down
	ACTOR_STANCE_SITTING, //!< ACTOR_STANCE_SITTING sitting
	ACTOR_STANCE_STAND_UP,//!< ACTOR_STANCE_STAND_UP stand up
	ACTOR_STANCE_RUN,     //!< ACTOR_STANCE_RUN run
	ACTOR_STANCE_TALK_A,  //!< ACTOR_STANCE_TALK_A type A of talking
	ACTOR_STANCE_TALK_B   //!< ACTOR_STANCE_TALK_B type B of talking
} ActorStance;

// -------------------------

/// \brief Actor's bounding types which
/// Social Force Model is able to use;
/// a model creates an artificial `inflation
/// layer` known from mobile robots navigation
typedef enum {
	ACTOR_BOUNDING_BOX = 0,	//!< ACTOR_BOUNDING_BOX a cuboid is created around the actor
	ACTOR_BOUNDING_CIRCLE, 	//!< ACTOR_BOUNDING_CIRCLE a cylinder is created around the actor
	ACTOR_BOUNDING_ELLIPSE, //!< ACTOR_BOUNDING_ELLIPSE an ellipsoidal cylinder is created around the actor
	ACTOR_NO_BOUNDING		//!< ACTOR_NO_BOUNDING no bounding is created around the actor
} ActorBoundingType;

// -------------------------

/// \brief Type of a visualization marker;
/// used to choose a proper topic to publish
/// a message on
/// DEPRECATED!
typedef enum {
	ACTOR_SOCIAL_FORCE = 0, //!< ACTOR_SOCIAL_FORCE
	ACTOR_SOCIAL_FORCE_GRID,//!< ACTOR_SOCIAL_FORCE_GRID
	ACTOR_BOUNDING,         //!< ACTOR_BOUNDING
	ACTOR_MODEL_CLOSEST, 	//!< ACTOR_MODEL_CLOSEST
} ActorVisType;

// -------------------------

/// \brief Block of enums connected with actor::ros_interface::Stream class;
/// according to type, a proper message type is published;
/// due to the fact that all publishers are stored in std::vector container
/// the number, which enum expands, must be UNIQUE (based on number the proper
/// publisher will be chosen);
/// this solution is marked as FIXME

/// \brief A single visualization_msgs::Marker IDs
typedef enum {
	ACTOR_MARKER_BOUNDING = 0u,		//!< ACTOR_MARKER_BOUNDING a marker around an actor
	ACTOR_MARKER_INTERNAL_VECTOR,   //!< ACTOR_MARKER_SF_VECTOR single arrow indicating a social force
	ACTOR_MARKER_INTERACTION_VECTOR,//!< ACTOR_MARKER_INTERACTION_VECTOR
	ACTOR_MARKER_SOCIAL_VECTOR,     //!< ACTOR_MARKER_SOCIAL_VECTOR
	ACTOR_MARKER_COMBINED_VECTOR    //!< ACTOR_MARKER_COMBINED_VECTOR
} ActorMarkerType;

// -------------------------

/// \brief An visualization_msgs::MarkerArray topics IDs
typedef enum {
	ACTOR_MARKER_ARRAY_CLOSEST_POINTS = 100u,//!< ACTOR_MARKER_ARRAY_CLOSEST_POINTS lines created from other objects points connected with actor's bounding
	ACTOR_MARKER_ARRAY_GRID                  //!< ACTOR_MARKER_ARRAY_GRID an arrow in each grid point indicating a social force
} ActorMarkerArrayType;

// -------------------------

/// \brief Transform frames IDs
typedef enum {
	ACTOR_TF_SELF = 200u,
	ACTOR_TF_CHECKPOINT,
	ACTOR_TF_TARGET
} ActorTfType;

// -------------------------

typedef enum {
	ACTOR_NAV_PATH = 300u,
} ActorNavMsgType;

// -------------------------

/// \brief View-oriented text
typedef enum {
	ACTOR_MARKER_TEXT_BEH = 400u,
} ActorMarkerTextType;

// -------------------------

} /* namespace actor */

#endif /* INCLUDE_ACTOR_ENUMS_H_ */
