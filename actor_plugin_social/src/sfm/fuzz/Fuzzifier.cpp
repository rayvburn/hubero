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
Fuzzifier::Fuzzifier(): level_(FUZZY_LEVEL_NONE), direction_(FUZZ_DIR_UNKNOWN),
		location_(FUZZ_LOCATION_UNKNOWN), object_dir_relative_angle_(0.0),
		vels_relative_angle_(0.0), d_alpha_beta_len_(0.0) {}

// ------------------------------------------------------------------- //
void Fuzzifier::setObjectVelocity(const ignition::math::Vector3d &vel_beta) {
	vel_beta_ = vel_beta;
}
// ------------------------------------------------------------------- //
void Fuzzifier::setObjectVelocity(const double &yaw_beta, const double &speed) {

	// create new velocity vector based on orientation and speed
	ignition::math::Vector3d vel_beta;
	vel_beta.X( speed*cos(yaw_beta) - speed*sin(yaw_beta) );
	vel_beta.Y( speed*sin(yaw_beta) + speed*cos(yaw_beta) );
	vel_beta.Z(0.0);
	setObjectVelocity(vel_beta);

}
// ------------------------------------------------------------------- //
void Fuzzifier::setVelsRelativeAngle(const double &vels_relative_angle) {
	vels_relative_angle_ = vels_relative_angle;
}
// ------------------------------------------------------------------- //
void Fuzzifier::setVelsRelativeAngle(const double &vel_alpha_angle, const double &vel_beta_angle) {

	ignition::math::Angle diff;
	diff.Radian(vel_beta_angle - vel_alpha_angle);
	diff.Normalize();
	setVelsRelativeAngle(diff.Radian());

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

	computeFuzzLocation(); // this must be called before direction computation
	computeFuzzDirection();
	computeFuzzLevel();
	return (true);

}

// ------------------------------------------------------------------- //

void Fuzzifier::resetParameters() {

	location_ = FUZZ_LOCATION_UNKNOWN;
	direction_ = FUZZ_DIR_UNKNOWN;
	level_ = FUZZY_LEVEL_NONE;

	// after `isApplicable()` call let's reset `vel_beta_` (by zeroing
	// velocity vector), which is used to check if object is dynamic
	vel_beta_ = ignition::math::Vector3d();

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

	// TODO: front-left, front-right, back-left, back-right
	if ( std::fabs(object_dir_relative_angle_) <= IGN_DTOR(10) ) {
		location_ = FUZZ_LOCATION_FRONT;
	} else if ( std::fabs(object_dir_relative_angle_) >= IGN_DTOR(100) ) {
		location_ = FUZZ_LOCATION_BACK;
	} else if ( object_dir_relative_angle_ > IGN_DTOR(10) ) {
		location_ = FUZZ_LOCATION_LEFT;
	} else if ( object_dir_relative_angle_ < IGN_DTOR(-10) ) {
		location_ = FUZZ_LOCATION_RIGHT;
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
			direction_ = FUZZ_DIR_PERP_OUT;
		} else {
			direction_ = FUZZ_DIR_PERP_CROSS;
		}
	} else if ( vels_relative_angle_ < IGN_DTOR(-10) ) {
		// this can be determined knowing the location
		if ( location_ == FUZZ_LOCATION_LEFT ) {
			direction_ = FUZZ_DIR_PERP_CROSS;
		} else {
			direction_ = FUZZ_DIR_PERP_OUT;
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
