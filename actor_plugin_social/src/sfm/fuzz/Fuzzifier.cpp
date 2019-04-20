/*
 * Fuzzifier.cpp
 *
 *  Created on: Apr 19, 2019
 *      Author: rayvburn
 */

#include "sfm/fuzz/Fuzzifier.h"

#include <math.h> 					// std::fabs()
#include <ignition/math/Helpers.hh> // IGN_PI_2
#include <ignition/math/Angle.hh>	// IGN_DTOR
#include <string> 					// debugging

namespace sfm {
namespace fuzz {

// ------------------------------------------------------------------- //

// initialization list
Fuzzifier::Fuzzifier(): vels_angle_set_(false), dist_set_(false), dir_angle_set_(false),
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

	// -----------------------------------------------------------
	std::cout << "Fuzzifier\n";
	std::cout << "\tvels_angle_set: " << vels_angle_set_ << "\tvels_relative_angle: " << vels_relative_angle_ << std::endl;
	std::cout << "\tdist_set: " << dist_set_ << "\t\tdist_between_objects: " << dist_between_objects_ << std::endl;
	std::cout << "\tdir_angle_set: " << dir_angle_set_ << "\tto_object_dir_relative_angle: " << to_object_dir_relative_angle_ << std::endl;
	std::cout << "\tsf_set: " << sf_set_ << "\t\tsf_vector_dir_angle: " << sf_vector_dir_angle_ << std::endl;
	// -----------------------------------------------------------

	bool detected = false;
	/* seems like a presence of the SFM_CONDITION_DYNAMIC_OBJECT_RIGHT condition
	 * could be investigated */

	/* -IGN_PI_2 -> -90 degress
	 * when `to_object_dir_relative_angle_` is equal to this value,
	 * one could say that object's relative position is straight
	 * on the right-hand side - a maximum case */
	bool is_on_the_right = false;

	// NOTE: the angle range isn't symmetrical
	if ( to_object_dir_relative_angle_ <= IGN_DTOR(-20) && to_object_dir_relative_angle_ >= IGN_DTOR(-120) ) {
		is_on_the_right = true;
		condition_ = SFM_CONDITION_DYNAMIC_OBJECT_RIGHT;
	}

//	// TODO: vels_relative_angle_ calculations
//	if ( is_on_the_right ) {
//		vels_relative_angle_
//	}

	// check if a condition was met
	if ( condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT ) {

		/* set level according to distance (NOTE: bounding box'es
		 * size is already taken into consideration (finding
		 * closest points) */
		if ( dist_between_objects_ <= 0.2 ) {
			level_ = FUZZY_LEVEL_EXTREME;
		} else if ( dist_between_objects_ <= 0.5 ) {
			level_ = FUZZY_LEVEL_HIGH;
		} else if ( dist_between_objects_ <= 1.0 ) {
			level_ = FUZZY_LEVEL_MEDIUM;
		} else if ( dist_between_objects_ <= 2.5 ) {
			level_ = FUZZY_LEVEL_LOW;
		} else {
			level_ = FUZZY_LEVEL_UNKNOWN;
		}

	}

	std::string level_txt, condition_txt;
	if ( condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT ) { condition_txt = "SFM_CONDITION_DYNAMIC_OBJECT_RIGHT"; } else if ( condition_ == SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE ) { condition_txt = "SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE"; } else { condition_txt = "SFM_CONDITION_UNKNOWN"; };
	if ( level_ == FUZZY_LEVEL_EXTREME ) { level_txt = "FUZZY_LEVEL_EXTREME"; } else if ( level_ == FUZZY_LEVEL_HIGH ) { level_txt = "FUZZY_LEVEL_HIGH"; } else if ( level_ == FUZZY_LEVEL_MEDIUM ) { level_txt = "FUZZY_LEVEL_MEDIUM"; } else if ( level_ == FUZZY_LEVEL_LOW ) { level_txt = "FUZZY_LEVEL_LOW"; } else { level_txt = "FUZZY_LEVEL_UNKNOWN"; };
	std::cout << "\t" << condition_txt << "\t" << level_txt << std::endl;

	/* this is calculated in separate step - when single interaction is investigated
	 * a total social force is not calculated yet */
	// FIXME logic to choose first or second
//	if ( !sf_set_ && !dir_angle_set_ ) {
//		std::cout << "Fuzzifier isConditionDetected() - no SF nor dist_rel_angle set!\n\n";
//		return (false);
//	}

	/* seems like a presence of the SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE condition
	 * could be investigated too */






	return (true);

}

// ------------------------------------------------------------------- //

void Fuzzifier::resetParameters() {

	condition_ = SFM_CONDITION_UNKNOWN;
	level_ = FUZZY_LEVEL_UNKNOWN;

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
