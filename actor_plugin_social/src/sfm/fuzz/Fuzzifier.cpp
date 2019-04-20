/*
 * Fuzzifier.cpp
 *
 *  Created on: Apr 19, 2019
 *      Author: rayvburn
 */

#include "sfm/fuzz/Fuzzifier.h"

namespace sfm {
namespace fuzz {

// ------------------------------------------------------------------- //

// initialization list
Fuzzifier::Fuzzifier(): dist_set_(false), dir_angle_set_(false), vels_angle_set_(false),
		sf_set_(false), vels_relative_angle_(0.0), dist_between_objects_(0.0),
		to_object_dir_relative_angle_(0.0), sf_vector_dir_angle_(0.0),
		condition_(SFM_CONDITION_UNKNOWN), level_(FUZZY_LEVEL_UNKNOWN)
{ }

// ------------------------------------------------------------------- //

void Fuzzifier::setDistanceVectorLength(const double &d_alpha_beta_len) {
	dist_between_objects_ = d_alpha_beta_len;
	dist_set_ = true;
}

// ------------------------------------------------------------------- //

void Fuzzifier::setToObjectDirectionRelativeAngle(const double &to_object_dir_relative_angle) {
	to_object_dir_relative_angle_ = to_object_dir_relative_angle;
	dir_angle_set_ = true;
}

// ------------------------------------------------------------------- //

void Fuzzifier::setVelocitiesRelativeAngle(const double &vels_relative_angle) {
	vels_relative_angle_ = vels_relative_angle;
	vels_angle_set_ = true;
}

// ------------------------------------------------------------------- //

void Fuzzifier::setSocialForce(const ignition::math::Vector3d &social_force) {
	sf_vector_dir_angle_ = std::atan2( social_force.Y(), social_force.X() );
	sf_set_ = true;
}

// ------------------------------------------------------------------- //

bool Fuzzifier::isConditionDetected() {

	// rough estimation what the input set should consist of
	// SFM_CONDITION_DYNAMIC_OBJECT_RIGHT 		- needed d_alpha_beta, v_alpha and v_beta
	// SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE - needed d_alpha_beta and social force

	if ( !vels_angle_set_ && (!dist_set_ || !dir_angle_set_)) {
		std::cout << "Fuzzifier isConditionDetected() - no VEL nor dist param set!\n\n";
		return (false);
	}

	/* seems like a presence of the SFM_CONDITION_DYNAMIC_OBJECT_RIGHT condition
	 * could be investigated */



	if ( !sf_set_ && !dir_angle_set_ ) {
		std::cout << "Fuzzifier isConditionDetected() - no SF nor dist_rel_angle set!\n\n";
		return (false);
	}

	/* seems like a presence of the SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE condition
	 * could be investigated too */






	return (true);

}

// ------------------------------------------------------------------- //

void Fuzzifier::resetParameters() {
	// no need to reset double-typed variables
	dist_set_ = false;
	dir_angle_set_ = false;
	vels_angle_set_ = false;
	sf_set_ = false;
}

// ------------------------------------------------------------------- //

Fuzzifier::~Fuzzifier() { }

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
