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
#include <sfm/core/SFMDebug.h>

namespace sfm {
namespace fuzz {

// ------------------------------------------------------------------- //

// initialization list
Fuzzifier::Fuzzifier(): vel_object_set_(false), vels_angle_set_(false), dist_set_(false),
		dir_angle_set_(false), sf_set_(false), vels_relative_angle_(0.0),
		dist_between_objects_(0.0), to_object_dir_relative_angle_(0.0),
		sf_vector_dir_angle_(0.0), condition_(SFM_CONDITION_UNKNOWN),
		level_(FUZZY_LEVEL_UNKNOWN)
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

void Fuzzifier::setOtherObjectVelocity(const ignition::math::Vector3d &object_vel) {
	object_vel_ = object_vel;
	vel_object_set_ = true;
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

	// predicates for flow control in this function
	bool check_object_on_the_right = false;
	bool check_force_direction = false;

	#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
	std::cout << SfmDebugGetCurrentActorName() << "\t" << SfmDebugGetCurrentObjectName() << "\t";
	#endif

	// only 1 condition checked at once
	if ( isActivePredicateForceDirection() ) {

		/* this is computed in a separate step - while a single interaction is investigated
		 * (i.e. `is object on the right` check) a total social force has not been
		 * calculated yet */
		#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
		std::cout << "Fuzzifier isConditionDetected() - force direction check!\n";
		#endif
		check_force_direction = true;

	} else if ( isActivePredicateObjectRight() ) {

		#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
		std::cout << "Fuzzifier isConditionDetected() - object on the right check!\n";
		#endif
		check_object_on_the_right = true;

	} else {

		#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
		std::cout << "Fuzzifier isConditionDetected() - NO PARAMS SET or OBJECT STATIC!\n\n";
		#endif
		return (false);

	}

	// -----------------------------------------------------------
//	std::cout << "Fuzzifier\n";
//	std::cout << "\tvels_angle_set: " << vels_angle_set_ << "\tvels_relative_angle: " << vels_relative_angle_ << "\t| deg:" << IGN_RTOD(vels_relative_angle_) << std::endl;
//	std::cout << "\tdist_set: " << dist_set_ << "\t\tdist_between_objects: " << dist_between_objects_ << std::endl;
//	std::cout << "\tdir_angle_set: " << dir_angle_set_ << "\tto_object_dir_relative_angle: " << to_object_dir_relative_angle_ << "\t| deg:" << IGN_RTOD(to_object_dir_relative_angle_) << std::endl;
//	std::cout << "\tsf_set: " << sf_set_ << "\t\tsf_vector_dir_angle: " << sf_vector_dir_angle_ << "\t| deg:" << IGN_RTOD(sf_vector_dir_angle_) << std::endl;
	// -----------------------------------------------------------

	bool detected = false;

	/* check if presence of the SFM_CONDITION_DYNAMIC_OBJECT_RIGHT
	 * condition could be investigated */
	if ( check_object_on_the_right ) {

		// helper flag
		bool is_on_the_right = false;

		/* -IGN_PI_2 -> -90 degrees
		 * when `to_object_dir_relative_angle_` is equal to this value,
		 * one could say that object's relative position is straight
		 * on the right-hand side - a maximum case */

		// NOTE: the angle range isn't symmetrical
		if ( to_object_dir_relative_angle_ <= IGN_DTOR(-20) && to_object_dir_relative_angle_ >= IGN_DTOR(-120) ) {
			is_on_the_right = true;
			condition_ = SFM_CONDITION_DYNAMIC_OBJECT_RIGHT;
			detected = true;
		}

		/* if an object detected on the right, then check
		 * if it's moving in perpendicular direction */
		if ( is_on_the_right ) {
			if ( vels_relative_angle_ >= IGN_DTOR(45) && vels_relative_angle_ <= IGN_DTOR(135) ) {
				condition_ = SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR;
				// no need to set `detected` here - already done above
			}
		}

		// check if a condition was met
		/* object on the right doesn't mean anything but when person's inertia
		 * is considered, then one should notice that object on the right also
		 * means that currently investigated person is moving ... */
		if ( condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT ||
			 condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR ) {

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
	} /* check_object_on_the_right */
	else if ( check_force_direction ) {


		// FIXME logic to choose first or second
//		if ( !sf_set_ && !dir_angle_set_ ) {
//			std::cout << "Fuzzifier isConditionDetected() - no SF nor dist_rel_angle set!\n\n";
//			return (false);
//		}

		/* seems like a presence of the SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE condition
		 * could be investigated too */

	} /* check_force_direction */


	#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
	// when one of the predicates is true then print debug
	if ( check_object_on_the_right || check_force_direction ) {
		std::string level_txt, condition_txt;
		if ( condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT ) { condition_txt = "SFM_CONDITION_DYNAMIC_OBJECT_RIGHT"; } else if ( condition_ == SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE ) { condition_txt = "SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE"; } else if ( condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR ) { condition_txt = "SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR"; } else { condition_txt = "SFM_CONDITION_UNKNOWN"; };
		if ( level_ == FUZZY_LEVEL_EXTREME ) { level_txt = "FUZZY_LEVEL_EXTREME"; } else if ( level_ == FUZZY_LEVEL_HIGH ) { level_txt = "FUZZY_LEVEL_HIGH"; } else if ( level_ == FUZZY_LEVEL_MEDIUM ) { level_txt = "FUZZY_LEVEL_MEDIUM"; } else if ( level_ == FUZZY_LEVEL_LOW ) { level_txt = "FUZZY_LEVEL_LOW"; } else { level_txt = "FUZZY_LEVEL_UNKNOWN"; };
		std::cout << "\t" << condition_txt << "\t" << level_txt << std::endl;
	}
	#endif

	return (detected);

}

// ------------------------------------------------------------------- //

void Fuzzifier::resetParameters() {

	condition_ = SFM_CONDITION_UNKNOWN;
	level_ = FUZZY_LEVEL_UNKNOWN;

	// no need to reset double-typed variables
	vel_object_set_ = false;
	dist_set_ = false;
	dir_angle_set_ = false;
	vels_angle_set_ = false;
	sf_set_ = false;

}

// ------------------------------------------------------------------- //

std::tuple<sfm::fuzz::SocialCondition, sfm::fuzz::FuzzyLevel> Fuzzifier::getSocialConditionAndLevel() const {
	return ( std::make_tuple(condition_, level_) );
}

// ------------------------------------------------------------------- //

bool Fuzzifier::isDynamicObject() {

	if ( object_vel_.Length() >= 1e-06 ) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

bool Fuzzifier::isActivePredicateObjectRight() {

	/*
	std::cout << "\nPREDICATE OBJECT RIGHT CHECK\n";
	std::cout << "\tvel_object_set: " << vel_object_set_ << std::endl;
	std::cout << "\tdist_set: " << dist_set_ << std::endl;
	std::cout << "\tdir_angle_set: " << dir_angle_set_ << std::endl;
	std::cout << "\tvels_angle_set: " << vels_angle_set_ << std::endl;
	std::cout << "\tsf_set: " << sf_set_ << std::endl;
	std::cout << "\tisDynamicObject: " << isDynamicObject() << std::endl;
	std::cout << "\n" << std::endl;
	*/

	if ( vel_object_set_ && isDynamicObject() && vels_angle_set_ && dist_set_ && dir_angle_set_ ) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

bool Fuzzifier::isActivePredicateForceDirection() {

	if ( sf_set_ && dir_angle_set_ ) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

Fuzzifier::~Fuzzifier() { }

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
