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

//// initialization list
Fuzzifier::Fuzzifier(): level_(FUZZY_LEVEL_NONE), direction_(FUZZ_DIR_UNKNOWN),
		location_(FUZZ_LOCATION_UNKNOWN), object_dir_relative_angle_(0.0),
		vels_relative_angle_(0.0), d_alpha_beta_len_(0.0) {}

// // ------------------------------------------------------------------- //
//void Fuzzifier::setInternalForce(const ignition::math::Vector3d &f_alpha) {
//	f_alpha_ = f_alpha;
//}
// // ------------------------------------------------------------------- //
//void Fuzzifier::setInteractionForceNorm(const ignition::math::Vector3d &f_alpha_beta_n) {
//	f_alpha_beta_n_ = f_alpha_beta_n;
//}
// // ------------------------------------------------------------------- //
//void Fuzzifier::setInteractionForcePerp(const ignition::math::Vector3d &f_alpha_beta_p) {
//	f_alpha_beta_p_ = f_alpha_beta_p;
//}
// ------------------------------------------------------------------- //
void Fuzzifier::setObjectVelocity(const ignition::math::Vector3d &vel_beta) {
	vel_beta_ = vel_beta;
}
// ------------------------------------------------------------------- //
void Fuzzifier::setVelsRelativeAngle(const double &vels_relative_angle) {
	vels_relative_angle_ = vels_relative_angle;
}
// ------------------------------------------------------------------- //
void Fuzzifier::setObjectDirRelativeAngle(const double &object_dir_relative_angle) {
	object_dir_relative_angle_ = object_dir_relative_angle;
}
// ------------------------------------------------------------------- //
void Fuzzifier::setDistanceVectorLength(const double &d_alpha_beta_len) {
	d_alpha_beta_len_ = d_alpha_beta_len;
}

// ------------------------------------------------------------------- //

bool Fuzzifier::isApplicable() {

	// only dynamic objects are considered in the `social` extension
	if ( !isDynamicObject() ) {
		return (false);
	}


//	// check relative angle
//	if ( 		object_dir_relative_angle_ >= IGN_DTOR(0) 	&& object_dir_relative_angle_ < IGN_DTOR(90) ) {
//
//		/* First quadrant: 0-90 degrees */
//
//	} else if ( object_dir_relative_angle_ >= IGN_DTOR(90) 	&& object_dir_relative_angle_ <= IGN_DTOR(180) ) {
//
//		/* Second quadrant: 90-180 degrees */
//
//	} else if ( object_dir_relative_angle_ <  IGN_DTOR(0) 	&& object_dir_relative_angle_ >= IGN_DTOR(-90) ) {
//
//		/* Third quadrant: 0-(-90) degrees */
//
//	} else if ( object_dir_relative_angle_ <  IGN_DTOR(-90) && object_dir_relative_angle_ >= IGN_DTOR(-180) ) {
//
//		/* Fourth quadrant: (-90)-(-180) degrees */
//
//	} else {
//
//		/* something went wrong */
//
//	}

	computeFuzzLocation(); // this must be called before direction computation
	computeFuzzDirection();
	computeFuzzLevel();
	return (true);

}

// ------------------------------------------------------------------- //

//bool Fuzzifier::isConditionDetected() {
//
//	// rough estimation what the input set should consist of
//	// SFM_CONDITION_DYNAMIC_OBJECT_RIGHT 		- needed d_alpha_beta, v_alpha and v_beta
//	// SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE - needed d_alpha_beta and social force
//
//	// predicates for flow control in this function
//	bool check_object_on_the_right = false;
//	bool check_force_direction = false;
//
//	#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
//	std::cout << SfmDebugGetCurrentActorName() << "\t" << SfmDebugGetCurrentObjectName() << "\t";
//	#endif
//
//	// only 1 condition checked at once
//	if ( isActivePredicateForceDirection() ) {
//
//		/* this is computed in a separate step - while a single interaction is investigated
//		 * (i.e. `is object on the right` check) a total social force has not been
//		 * calculated yet */
//		#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
//		std::cout << "Fuzzifier isConditionDetected() - force direction check!\n";
//		#endif
//		check_force_direction = true;
//
//	} else if ( isActivePredicateObjectRight() ) {
//
//		#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
//		std::cout << "Fuzzifier isConditionDetected() - object on the right check!\n";
//		#endif
//		check_object_on_the_right = true;
//
//	} else {
//
//		#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
//		std::cout << "Fuzzifier isConditionDetected() - NO PARAMS SET or OBJECT STATIC!\n\n";
//		#endif
//		return (false);
//
//	}
//
//	// -----------------------------------------------------------
////	std::cout << "Fuzzifier\n";
////	std::cout << "\tvels_angle_set: " << vels_angle_set_ << "\tvels_relative_angle: " << vels_relative_angle_ << "\t| deg:" << IGN_RTOD(vels_relative_angle_) << std::endl;
////	std::cout << "\tdist_set: " << dist_set_ << "\t\tdist_between_objects: " << dist_between_objects_ << std::endl;
////	std::cout << "\tdir_angle_set: " << dir_angle_set_ << "\tto_object_dir_relative_angle: " << to_object_dir_relative_angle_ << "\t| deg:" << IGN_RTOD(to_object_dir_relative_angle_) << std::endl;
////	std::cout << "\tsf_set: " << sf_set_ << "\t\tsf_vector_dir_angle: " << sf_vector_dir_angle_ << "\t| deg:" << IGN_RTOD(sf_vector_dir_angle_) << std::endl;
//	// -----------------------------------------------------------
//
//	bool detected = false;
//
//	/* check if presence of the SFM_CONDITION_DYNAMIC_OBJECT_RIGHT
//	 * condition could be investigated */
//	if ( check_object_on_the_right ) {
//
//		// helper flag
//		bool is_on_the_right = false;
//
//		/* -IGN_PI_2 -> -90 degrees
//		 * when `to_object_dir_relative_angle_` is equal to this value,
//		 * one could say that object's relative position is straight
//		 * on the right-hand side - a maximum case */
//
//		// NOTE: the angle range isn't symmetrical
//		if ( to_object_dir_relative_angle_ <= IGN_DTOR(-20) && to_object_dir_relative_angle_ >= IGN_DTOR(-120) ) {
//			is_on_the_right = true;
//			condition_ = SFM_CONDITION_DYNAMIC_OBJECT_RIGHT;
//			detected = true;
//		}
//
//		/* if an object detected on the right, then check
//		 * if it's moving in perpendicular direction */
//		if ( is_on_the_right ) {
//			if ( vels_relative_angle_ >= IGN_DTOR(45) && vels_relative_angle_ <= IGN_DTOR(135) ) {
//				condition_ = SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR;
//				// no need to set `detected` here - already done above
//			}
//		}
//
//		// check if a condition was met
//		/* object on the right doesn't mean anything but when person's inertia
//		 * is considered, then one should notice that object on the right also
//		 * means that currently investigated person is moving ... */
//		if ( condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT ||
//			 condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR ) {
//
//			/* set level according to distance (NOTE: bounding box'es
//			 * size is already taken into consideration (finding
//			 * closest points) */
//			if ( dist_between_objects_ <= 0.2 ) {
//				level_ = FUZZY_LEVEL_EXTREME;
//			} else if ( dist_between_objects_ <= 0.5 ) {
//				level_ = FUZZY_LEVEL_HIGH;
//			} else if ( dist_between_objects_ <= 1.0 ) {
//				level_ = FUZZY_LEVEL_MEDIUM;
//			} else if ( dist_between_objects_ <= 2.5 ) {
//				level_ = FUZZY_LEVEL_LOW;
//			} else {
//				level_ = FUZZY_LEVEL_UNKNOWN;
//			}
//
//		}
//	} /* check_object_on_the_right */
//	else if ( check_force_direction ) {
//
//
//		// FIXME logic to choose first or second
////		if ( !sf_set_ && !dir_angle_set_ ) {
////			std::cout << "Fuzzifier isConditionDetected() - no SF nor dist_rel_angle set!\n\n";
////			return (false);
////		}
//
//		/* seems like a presence of the SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE condition
//		 * could be investigated too */
//
//	} /* check_force_direction */
//
//
//	#ifdef DEBUG_FUZZIFIER_CONDITION_INFO
//	// when one of the predicates is true then print debug
//	if ( check_object_on_the_right || check_force_direction ) {
//		std::string level_txt, condition_txt;
//		if ( condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT ) { condition_txt = "SFM_CONDITION_DYNAMIC_OBJECT_RIGHT"; } else if ( condition_ == SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE ) { condition_txt = "SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE"; } else if ( condition_ == SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR ) { condition_txt = "SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR"; } else { condition_txt = "SFM_CONDITION_UNKNOWN"; };
//		if ( level_ == FUZZY_LEVEL_EXTREME ) { level_txt = "FUZZY_LEVEL_EXTREME"; } else if ( level_ == FUZZY_LEVEL_HIGH ) { level_txt = "FUZZY_LEVEL_HIGH"; } else if ( level_ == FUZZY_LEVEL_MEDIUM ) { level_txt = "FUZZY_LEVEL_MEDIUM"; } else if ( level_ == FUZZY_LEVEL_LOW ) { level_txt = "FUZZY_LEVEL_LOW"; } else { level_txt = "FUZZY_LEVEL_UNKNOWN"; };
//		std::cout << "\t" << condition_txt << "\t" << level_txt << std::endl;
//	}
//	#endif
//
//	return (detected);
//
//}

// ------------------------------------------------------------------- //

void Fuzzifier::resetParameters() {

	location_ = FUZZ_LOCATION_UNKNOWN;
	direction_ = FUZZ_DIR_UNKNOWN;
	level_ = FUZZY_LEVEL_NONE;

	// TODO: adjust after changes in `isApplicable()`, to check if object
	// is dynamic zeroing velocity vector is sufficient
	vel_beta_ = ignition::math::Vector3d();

//	vels_relative_angle_ = 0.0;
//	object_dir_relative_angle_ = 0.0;
//	d_alpha_beta_len_ = 0.0;

}

// ------------------------------------------------------------------- //

std::tuple<FuzzLocation, FuzzDirection, FuzzLevel> Fuzzifier::getFuzzyState() const {
	return ( std::make_tuple(location_, direction_, level_) );
}

// ------------------------------------------------------------------- //

inline bool Fuzzifier::isDynamicObject() {

	if ( vel_beta_.Length() >= 1e-06 ) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

void Fuzzifier::computeFuzzLocation() {

	if ( std::fabs(object_dir_relative_angle_) <= IGN_DTOR(10) ) {
		location_ = FUZZ_LOCATION_FRONT;
	} else if ( std::fabs(object_dir_relative_angle_) >= IGN_DTOR(160) ) {
		location_ = FUZZ_LOCATION_BACK;
	} else if ( object_dir_relative_angle_ > IGN_DTOR(10) ) {
//		location_ = FUZZ_LOCATION_LEFT;
		location_ = FUZZ_LOCATION_RIGHT;
	} else if ( object_dir_relative_angle_ < IGN_DTOR(-10) ) {
//		location_ = FUZZ_LOCATION_RIGHT;
		location_ = FUZZ_LOCATION_LEFT;
	} else {
		location_ = FUZZ_LOCATION_UNKNOWN;
	}

}

// ------------------------------------------------------------------- //

void Fuzzifier::computeFuzzDirection() {

	// based on velocity vectors difference check whether Beta crosses Alpha's
	// direction or objects directions are equal/opposite
	if ( std::fabs(vels_relative_angle_) <= IGN_DTOR(10) ) {
		direction_ = FUZZ_DIR_EQUAL;
	} else if ( std::fabs(vels_relative_angle_) >= IGN_DTOR(160) ) {
		direction_ = FUZZ_DIR_OPPOSITE;
	} else if ( vels_relative_angle_ > IGN_DTOR(10) ) {
		// this can be determined knowing the location
		if ( location_ == FUZZ_LOCATION_LEFT ) {
			direction_ = FUZZ_DIR_PERP_CROSS;
		} else {
			direction_ = FUZZ_DIR_PERP_OUT;
		}
	} else if ( vels_relative_angle_ < IGN_DTOR(-10) ) {
		// this can be determined knowing the location
		if ( location_ == FUZZ_LOCATION_LEFT ) {
			direction_ = FUZZ_DIR_PERP_OUT;
		} else {
			direction_ = FUZZ_DIR_PERP_CROSS;
		}
	} else {
		direction_ = FUZZ_DIR_UNKNOWN;
	}

}

// ------------------------------------------------------------------- //

void Fuzzifier::computeFuzzLevel() {

	// differentiate level (intensity) of the social extension based on d_alpha_beta length
	if ( d_alpha_beta_len_ < 0.10 ) {
		level_ = FUZZY_LEVEL_EXTREME;
	} else if (d_alpha_beta_len_ < 0.25 ) {
		level_ = FUZZY_LEVEL_HIGH;
	} else if (d_alpha_beta_len_ < 0.40 ) {
		level_ = FUZZY_LEVEL_MEDIUM;
	} else if (d_alpha_beta_len_ < 0.60 ) {
		level_ = FUZZY_LEVEL_LOW;
	} else {
		level_ = FUZZY_LEVEL_NONE;
	}

}

// ------------------------------------------------------------------- //

Fuzzifier::~Fuzzifier() { }

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
