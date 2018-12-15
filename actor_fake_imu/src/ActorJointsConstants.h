/*
 * actor_joints_constants.h
 *
 *  Created on: Dec 11, 2018
 *      Author: rayvburn
 */

#ifndef SRC_ACTORJOINTSCONSTANTS_H_
#define SRC_ACTORJOINTSCONSTANTS_H_

// ----------------------------------------------------------------------------

/*
 * In the actor animation ::Hips joint is only rotating around the axis parallel
 * to the the world's Z axis. It does not change its orientation as the actor
 * is bending etc.
 */

// ----------------------------------------------------------------------------

const float EXTRA_RANGE = 0.03; // in radians

// - - - - - - - - - - - - - - - - - -

typedef enum {

	WALKING = 0u,
	STANDING,
	STANDING_BENT,
	SITTING,
	LYING

} ActorState;

// - - - - - - - - - - - - - - - - - -

typedef struct {

	// orientation values of a joint
	float roll_min;
	float roll_max;
	float pitch_min;
	float pitch_max;
	// yaw is fully dependent on the actor's hip rotation

} ActorJointConfig;

// - - - - - - - - - - - - - - - - - -

typedef struct {

	// orientation values of 2 joints
	ActorJointConfig lower_back;
	ActorJointConfig upper_leg;
	float extra_range;

} ActorStance;

// - - - - - - - - - - - - - - - - - -

typedef struct {
	float x;
	float y;
	float z;
} ActorPosition;

// - - - - - - - - - - - - - - - - - -

// values extracted from Gazebo under way of calibration process
const ActorStance WALKING_CONFIG       = 	{ {+1.85, +1.88, -0.08, +0.12},
											  {-1.80, -1.01, -0.18, +0.16},
											   EXTRA_RANGE };

const ActorStance STANDING_CONFIG 	   = 	{ {+1.70, +1.76, +0.07, +0.12},
											  {-1.56, -1.47, -0.06, +0.12},
											   EXTRA_RANGE };

// TODO
const ActorStance STANDING_BENT_CONFIG =	{ {0.0, 0.0, 0.0, 0.0},
											  {0.0, 0.0, 0.0, 0.0},
											   EXTRA_RANGE };	// stance often caused by an intensive stomach pain

const ActorStance SITTING_CONFIG 	   = 	{ {+2.26, +2.28, -0.06, -0.03},
											  {-0.64, -0.60, -0.06, -0.06},
											   EXTRA_RANGE };

// TODO
const ActorStance LYING_CONFIG 		   = 	{ {0.0, 0.0, 0.0, 0.0},
										  	  {0.0, 0.0, 0.0, 0.0},
											   EXTRA_RANGE };

// ----------------------------------------------------------------------------

#endif /* SRC_ACTORJOINTSCONSTANTS_H_ */
