/*
 * Enums.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_ENUMS_H_
#define INCLUDE_ACTOR_ENUMS_H_

namespace actor {

/// \brief Gazebo model ID for ActorPlugin obejcts
static constexpr unsigned int ACTOR_MODEL_TYPE_ID = 32771;

// -------------------------

typedef enum {
	ACTOR_STATE_ALIGN_TARGET = 0,
	ACTOR_STATE_STUCK,
	ACTOR_STATE_MOVE_AROUND,
	ACTOR_STATE_STOP_AND_STARE,
	ACTOR_STATE_FOLLOW_OBJECT,
	ACTOR_STATE_TELEOPERATION
} ActorState;

static const ActorState ACTOR_STATE_INITIAL = ACTOR_STATE_ALIGN_TARGET;

// -------------------------

typedef enum {
	ACTOR_STANCE_WALK = 0,
	ACTOR_STANCE_STAND,
	ACTOR_STANCE_LIE,
	ACTOR_STANCE_SIT_DOWN,
	ACTOR_STANCE_SITTING,
	ACTOR_STANCE_STAND_UP,
	ACTOR_STANCE_RUN,
	ACTOR_STANCE_TALK_A,
	ACTOR_STANCE_TALK_B
} ActorStance;

// -------------------------

typedef enum {
	ACTOR_BOUNDING_BOX = 0,
	ACTOR_BOUNDING_CIRCLE,
	ACTOR_BOUNDING_ELLIPSE
} ActorBoundingType;

// -------------------------

typedef enum {
	ACTOR_SOCIAL_FORCE = 0,
	ACTOR_SOCIAL_FORCE_GRID,
	ACTOR_BOUNDING,
	ACTOR_MODEL_CLOSEST, 		// i.e. ACTOR_MODEL_BOUNDINGS_CLOSEST_POINTS
} ActorVisType;

// -------------------------

/// \brief Block of enums connected with actor::ros_interface::Stream class;
/// according to type, a proper message type is published;
/// due to the fact that all publishers are stored in std::vector container
/// the number, which enum expands, must be UNIQUE (based on number the
/// publisher will be chosen);
/// this solution is marked as FIXME

typedef enum {
	ACTOR_MARKER_BOUNDING = 0u,
	ACTOR_MARKER_SF_VECTOR,
} ActorMarkerType;

// -------------------------

typedef enum {
	ACTOR_MARKER_ARRAY_CLOSEST_POINTS = 100u,
	ACTOR_MARKER_ARRAY_GRID
} ActorMarkerArrayType;

// -------------------------

typedef enum {
	ACTOR_TF_SELF = 200u,
	ACTOR_TF_TARGET
} ActorTfType;

// -------------------------

typedef enum {
	ACTOR_NAV_PATH = 300u,
} ActorNavMsgType;

// -------------------------

} /* namespace actor */

#endif /* INCLUDE_ACTOR_ENUMS_H_ */
