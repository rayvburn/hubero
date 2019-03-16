/*
 * Enums.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ENUMS_H_
#define INCLUDE_ENUMS_H_

// -------------------------

const unsigned int ACTOR_MODEL_TYPE = 32771;

// -------------------------

typedef enum {
	ACTOR_STANCE_WALK = 0,
	ACTOR_STANCE_STAND,
	ACTOR_STANCE_LIE,
} ActorStance;

// -------------------------

typedef enum {
	ACTOR_STATE_ALIGN_TARGET = 0,
	ACTOR_STATE_MOVE_AROUND,
	ACTOR_STATE_STOP_AND_STARE,
	ACTOR_STATE_FOLLOW_OBJECT,
	ACTOR_STATE_TELEOPERATION,
} ActorState;

// -------------------------

typedef enum {
	ACTOR_BOUNDING_BOX = 0,
	ACTOR_BOUNDING_CIRCLE,
	ACTOR_BOUNDING_ELLIPSE
} ActorBoundingType;

// -------------------------

#endif /* INCLUDE_ENUMS_H_ */
