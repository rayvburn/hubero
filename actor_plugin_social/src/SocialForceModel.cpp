/*
 * SocialForceModel.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#include <SocialForceModel.h>

namespace SocialForceModel {

SocialForceModel::SocialForceModel() {

	/* Algorithm PARAMETERS are:
	 * - relaxation time must be given here
	 * - kind of coefficient for attraction artificial potential field (decreases over time)
	 * - FOV
	 * - direction weight
	 * - max speed
	 */
}

double SocialForceModel::GetSocialForce() {

	/* Algorithm INPUTS are:
	 * - dt
	 * - goal reaching component (acceleration term)
	 * 		- target's pose
	 * 		- current actor's pose
	 * 		- actual speed
	 * 		- desired speed
	 * 		- relaxation time
	 * - dynamic object repulsion component (other people or obstacles)
	 * 		- object's pose
	 * 		- current actor's pose
	 * 		- object's speed
	 * - static object repulsion component (borders)
	 * 		- object's pose
	 * 		- current actor's pose
	 *	- object attraction component (other people or objects)
	 *		- field decrease factor
	 *		- object's pose
	 *		- current actor's pose
	 *
	 */

	return 0.00;

}

ignition::math::Vector3d SocialForceModel::CalculateNumGradient(ignition::math::Vector3d &_vector) {

	// Vector3D is just for convenience (to avoid conversion)
	// for 2 dimensional vectors numerical gradient is just a difference (y-x)
	ignition::math::Vector3d num_gradient(_vector.Y()-_vector.X(), _vector.Y()-_vector.X(), 0.0);
	return num_gradient;

}

SocialForceModel::~SocialForceModel() { }

}
