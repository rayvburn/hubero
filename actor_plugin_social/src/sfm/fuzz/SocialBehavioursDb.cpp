/*
 * SocialBehavioursDb.cpp
 *
 *  Created on: Sep 15, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/SocialBehavioursDb.h>
#include <ignition/math/Angle.hh>

namespace sfm {
namespace fuzz {

// ------------------------------------------------------------------- //

SocialBehavioursDb::SocialBehavioursDb(const double &social_force_strength)
	: social_force_strength_(social_force_strength), d_alpha_beta_length_(0.0), dir_alpha_(0.0)
{ }

// ------------------------------------------------------------------- //

void SocialBehavioursDb::setDistance(const double &d_alpha_beta) {
	d_alpha_beta_length_ = d_alpha_beta;
}

// ------------------------------------------------------------------- //

void SocialBehavioursDb::setDirection(const double &dir) {
	dir_alpha_ = dir;
}

// ------------------------------------------------------------------- //

SocialBehavioursDb::~SocialBehavioursDb() { }

// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
// -------PROTECTED-MEMBERS-SECTION----------------------------------- //
// ------------------------------------------------------------------- //
ignition::math::Vector3d SocialBehavioursDb::turnLeft() {

	ignition::math::Vector3d force;

	double dir_rot = findOrientation(dir_alpha_, 'l');
	force = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(500.0);

	force = setVectorLength(force, magnitude);

	return (force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::turnLeftAccelerate() {

	ignition::math::Vector3d force;

	// rotation section (turning)
	double dir_rot = findOrientation(dir_alpha_, 'l');
	force = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(500.0);

	force = setVectorLength(force, magnitude);

	// acceleration section
	magnitude *= 1.25;	// FIXME
	force = extendVector(force, dir_alpha_, magnitude);

	return (force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::goAlong() {

	// do nothing
	return (ignition::math::Vector3d());

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::accelerate() {

	ignition::math::Vector3d force;

	// acceleration section
	double magnitude = calculateVectorMagnitude(500.0);

	force = createDirVector(dir_alpha_);
	force = setVectorLength(force, magnitude);

	return (force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::turnRightAccelerate() {

	ignition::math::Vector3d force;

	// rotation section (turning)
	double dir_rot = findOrientation(dir_alpha_, 'r');
	force = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(500.0);

	force = setVectorLength(force, magnitude);

	// acceleration section
	//magnitude *= 1.25;
	force = extendVector(force, dir_alpha_, magnitude);

	return (force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::turnRight() {

	ignition::math::Vector3d force;

	double dir_rot = findOrientation(dir_alpha_, 'r');
	force = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(800.0); // 600.0

	force = setVectorLength(force, magnitude);

	return (force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::turnRightDecelerate() {

	ignition::math::Vector3d force;

	// rotation section (turning)
	double dir_rot = findOrientation(dir_alpha_, 'r');
	force = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(500.0);

	force = setVectorLength(force, magnitude);

	// deceleration section
	dir_rot = findOrientation(dir_alpha_, 'o');
	magnitude *= 1.25;
	force = extendVector(force, dir_rot, magnitude);

	return (force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::stop() {

	ignition::math::Vector3d force;

	// TODO: resulting force vector is needed
	double dir_rot = findOrientation(dir_alpha_, 'o');
	force = createDirVector(dir_rot);

	double magnitude = calculateVectorMagnitude(500.0);

	force = setVectorLength(force, magnitude);

	return (force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::decelerate() {

	ignition::math::Vector3d force;

	// acceleration section
	double magnitude = calculateVectorMagnitude(500.0);

	double dir_opp = findOrientation(dir_alpha_, 'o');
	force = createDirVector(dir_opp);
	force = setVectorLength(force, magnitude);

	return (force);

}

// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //
// -------PRIVATE-MEMBERS-SECTION------------------------------------- //
// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::createDirVector(const double &direction) const {

	// X = 1.0, Y = 0.0
	ignition::math::Vector3d v;
	v.X(cos(direction));
	v.Y(sin(direction));
	v.Z(0.0);

	return (v);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::setVectorLength(const ignition::math::Vector3d &v,
		const double &magnitude) const {

	ignition::math::Vector3d v_new = v.Normalized();
	v_new *= magnitude;
	return (v_new);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialBehavioursDb::extendVector(const ignition::math::Vector3d &v, const double &dir,
		const double &magnitude) const {

	// create unit vector pointing in the direction according to `dir`
	ignition::math::Vector3d v_ext = createDirVector(dir);

	// extend the vector according to magnitude
	v_ext *= magnitude;

	// sum up 2 vectors
	return (v_ext + v);

}

// ------------------------------------------------------------------- //

double SocialBehavioursDb::findOrientation(const double &dir, const char &side) const {

	ignition::math::Angle dir_new(dir);

	if ( side == 'l' ) {			// LEFT
		dir_new.Radian(dir_new.Radian() + IGN_PI_2);
	} else if ( side == 'r' ) {		// RIGHT
		dir_new.Radian(dir_new.Radian() - IGN_PI_2);
	} else if ( side == 'o' ) {		// OPPOSITE
		dir_new.Radian(dir_new.Radian() + IGN_PI);
	}

	dir_new.Normalize();
	return (dir_new.Radian());

}

// ------------------------------------------------------------------- //

double SocialBehavioursDb::calculateVectorMagnitude(const double &max) const {

	const double X_SOCIAL_RANGE = 4.0; 					// in meters

	// no social force vector is generated if obstacle is too far away
	if ( d_alpha_beta_length_ > X_SOCIAL_RANGE ) {
		return (0.0);
	}

	double a = -(max)/(X_SOCIAL_RANGE); 			// in fact (X_SOCIAL_RANGE_END - X_SOCIAL_RANGE_START) but the start is 0.0
	double y = a * d_alpha_beta_length_ + max; 		// form of a line equation for readability,
													// the independent variable is `d_alpha_beta_length_`
	return (y);

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
