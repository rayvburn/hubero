/*
 * Defuzzifier.cpp
 *
 *  Created on: Apr 20, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/Defuzzifier.h>
#include <ignition/math/Angle.hh>

namespace sfm {
namespace fuzz {

typedef enum {
	ASSERT_DIR_RIGHT = 0u,
	ASSERT_DIR_LEFT
} PerpAssertDir;

// ------------------------------------------------------------------- //
Defuzzifier::Defuzzifier(): location_(FUZZ_LOCATION_UNKNOWN), direction_(FUZZ_DIR_UNKNOWN), level_(FUZZY_LEVEL_NONE)
{}
// ------------------------------------------------------------------- //
void Defuzzifier::setInternalForce(const ignition::math::Vector3d &f_alpha) {
	f_alpha_ = f_alpha;
}
// ------------------------------------------------------------------- //
void Defuzzifier::setInteractionForceNorm(const ignition::math::Vector3d &f_alpha_beta_n) {
	f_alpha_beta_n_ = f_alpha_beta_n;
}
// ------------------------------------------------------------------- //
void Defuzzifier::setInteractionForcePerp(const ignition::math::Vector3d &f_alpha_beta_p) {
	f_alpha_beta_p_ = f_alpha_beta_p;
}
// ------------------------------------------------------------------- //
void Defuzzifier::setFuzzState(const FuzzLocation &location, const FuzzDirection &direction, const FuzzLevel &level) {
	location_ = location; direction_ = direction; level_ = level;
}
// ------------------------------------------------------------------- //
void Defuzzifier::setFuzzState(const std::tuple<FuzzLocation, FuzzDirection, FuzzLevel> &fuzz) {
	location_ = std::get<0>(fuzz); direction_ = std::get<1>(fuzz); level_ = std::get<2>(fuzz);
}
// ------------------------------------------------------------------- //
ignition::math::Vector3d Defuzzifier::defuzzifySocialForce() const {

	// actual `social` component of the SFM
	ignition::math::Vector3d social_force;

	// check whether Fuzzifier state is valid
	if ( location_ == FUZZ_LOCATION_UNKNOWN || direction_ == FUZZ_DIR_UNKNOWN || level_ == FUZZY_LEVEL_NONE ) {
		return (social_force);
	}

	// TODO: graphical representation of the algorithm
	switch (location_) {

	case(FUZZ_LOCATION_FRONT):

			if ( direction_ == FUZZ_DIR_EQUAL ) {
				// do nothing
				std::cout << "\tdefuzzify\tLOC_FRONT\tDIR_EQUAL" << std::endl;
			} else if ( direction_ == FUZZ_DIR_OPPOSITE ) {
				// do nothing, default SFM formulation should take care of slight turning right
//				social_force = assertPerpDirection(ASSERT_DIR_RIGHT);
				std::cout << "\tdefuzzify\tLOC_FRONT\tDIR_OPPOSITE" << std::endl;
			} else if ( direction_ == FUZZ_DIR_PERP_OUT ) {
//				social_force = assertPerpDirection(ASSERT_DIR_LEFT);	std::cout << "\tdefuzzify\tLOC_FRONT\tDIR_PERP_OUT" << std::endl;
				std::cout << "\tdefuzzify\tLOC_FRONT\tDIR_PERP_OUT" << std::endl;
			} else if ( direction_ == FUZZ_DIR_PERP_CROSS ) {
//				social_force = assertPerpDirection(ASSERT_DIR_RIGHT);	std::cout << "\tdefuzzify\tLOC_FRONT\tDIR_PERP_CROSS" << std::endl;
				std::cout << "\tdefuzzify\tLOC_FRONT\tDIR_PERP_CROSS" << std::endl;
			}

			break;

	case(FUZZ_LOCATION_LEFT):

			if ( direction_ == FUZZ_DIR_EQUAL ) {
				// do nothing
				std::cout << "\tdefuzzify\tLOC_LEFT\tDIR_EQUAL" << std::endl;
			} else if ( direction_ == FUZZ_DIR_OPPOSITE ) {
				// do nothing, default SFM formulation should take care of slight turning right
				std::cout << "\tdefuzzify\tLOC_LEFT\tDIR_OPPOSITE" << std::endl;
			} else if ( direction_ == FUZZ_DIR_PERP_OUT ) {
//				social_force = assertPerpDirection(ASSERT_DIR_RIGHT);	std::cout << "\tdefuzzify\tLOC_LEFT\tDIR_PERP_OUT" << std::endl;
				std::cout << "\tdefuzzify\tLOC_LEFT\tDIR_PERP_OUT" << std::endl;
			} else if ( direction_ == FUZZ_DIR_PERP_CROSS ) {
//				social_force = assertPerpDirection(ASSERT_DIR_RIGHT);	std::cout << "\tdefuzzify\tLOC_LEFT\tDIR_PERP_CROSS" << std::endl;
				std::cout << "\tdefuzzify\tLOC_LEFT\tDIR_PERP_CROSS" << std::endl;
			}

			break;

	case(FUZZ_LOCATION_BACK):

			if ( direction_ == FUZZ_DIR_EQUAL ) {
				/* do nothing */				std::cout << "\tdefuzzify\tLOC_BACK\tDIR_EQUAL" << std::endl;
			} else if ( direction_ == FUZZ_DIR_OPPOSITE ) {
				// default SFM formulation should take care of slight turning right
				/* do nothing */				std::cout << "\tdefuzzify\tLOC_BACK\tDIR_OPPOSITE" << std::endl;
			} else if ( direction_ == FUZZ_DIR_PERP_OUT ) {
				/* do nothing */				std::cout << "\tdefuzzify\tLOC_BACK\tDIR_PERP_OUT" << std::endl;
			} else if ( direction_ == FUZZ_DIR_PERP_CROSS ) {
				/* do nothing */				std::cout << "\tdefuzzify\tLOC_BACK\tDIR_PERP_CROSS" << std::endl;
			}

			break;

	case(FUZZ_LOCATION_RIGHT):

			if ( direction_ == FUZZ_DIR_EQUAL ) {
				/* do nothing */				std::cout << "\tdefuzzify\tLOC_RIGHT\tDIR_EQUAL" << std::endl;
			} else if ( direction_ == FUZZ_DIR_OPPOSITE ) {
				social_force = passRight(); 	std::cout << "\tdefuzzify\tLOC_RIGHT\tDIR_OPPOSITE-passRight" << std::endl;
			} else if ( direction_ == FUZZ_DIR_PERP_OUT ) {
				social_force = passLeft();	 	std::cout << "\tdefuzzify\tLOC_RIGHT\tDIR_PERP_OUT-passLeft" << std::endl;
			} else if ( direction_ == FUZZ_DIR_PERP_CROSS ) {
				social_force = letPass();	 	std::cout << "\tdefuzzify\tLOC_RIGHT\tDIR_PERP_CROSS-letPass" << std::endl;
			}

			break;

	}

	return (social_force);

}

// ------------------------------------------------------------------- //

void Defuzzifier::reset() {
	slowed_down_ = false;
}

// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::passRight() const {

	ignition::math::Vector3d social_force;

	social_force = assertPerpDirection(ASSERT_DIR_RIGHT); // , true

	if ( !slowed_down_ ) {
		social_force += weakenInternalForce();
	}

	return (social_force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::passLeft() const {

	ignition::math::Vector3d social_force;

	social_force = assertPerpDirection(ASSERT_DIR_LEFT); // , true

	if ( !slowed_down_ ) {
		social_force += weakenInternalForce();
	}

	return (social_force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::letPass() const {

	ignition::math::Vector3d social_force;

	social_force = assertPerpDirection(ASSERT_DIR_RIGHT); // , true

	if ( !slowed_down_ ) {
		social_force += weakenInternalForce();
	}

	return (social_force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::assertPerpDirection(const unsigned int &dir, bool slow_down) const {

	ignition::math::Vector3d social_force;

	// Check whether direction of the perpendicular component
	// of f_alpha_beta points to the DIR. If it doesn't
	// then rotate it appropriately.
	double n_alpha_dir = std::atan2(f_alpha_beta_n_.Y(), f_alpha_beta_n_.X());
	double p_alpha_dir = std::atan2(f_alpha_beta_p_.Y(), f_alpha_beta_p_.X());
	ignition::math::Angle dir_diff(p_alpha_dir - n_alpha_dir);
	dir_diff.Normalize();

	// It is assumed that perpendicular component is almost perfectly perpendicular indeed.
	// Check ~90 degree angle condition, if it is not fulfilled then likely direction
	// of the perpendicular component is the opposite.
	if ( dir_diff.Radian() > IGN_DTOR(87) && dir_diff.Radian() < IGN_DTOR(93) ) {

		// `dir_diff` is ~(+90) -> perpendicular component points to the LEFT
		if ( dir == ASSERT_DIR_LEFT ) {
			// do nothing
		} else if ( dir == ASSERT_DIR_RIGHT ) {
			// rotate
			social_force = rotateVector(f_alpha_beta_p_, IGN_PI);
			// perpendicular component has wrong orientation, let's (over)compensate it
			social_force *= 2.0;
		}

	} else {

		// `dir_diff` is ~(-90) -> perpendicular component points to the RIGHT
		if ( dir == ASSERT_DIR_LEFT ) {
			// rotate
			social_force = rotateVector(f_alpha_beta_p_, IGN_PI);
			// perpendicular component has wrong orientation, let's (over)compensate it
			social_force *= 2.0;
		} else if ( dir == ASSERT_DIR_RIGHT ) {
			// do nothing
		}

	}

	// check `slow_down` flag
	if ( slow_down ) {
		// rotated component must be multiplied (>1.00) to have an opposite effect
		// (relative to perpendicular component), otherwise it will just be `equalized`
		switch (level_) {

		case(FUZZY_LEVEL_LOW):
				social_force *= 1.00;
				break;
		case(FUZZY_LEVEL_MEDIUM):
				social_force *= 1.50;
				break;
		case(FUZZY_LEVEL_HIGH):
				social_force *= 2.25;
				break;
		case(FUZZY_LEVEL_EXTREME):
				social_force *= 3.00;
				break;

		}
	}

	return (social_force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::weakenInternalForce() const {

	ignition::math::Vector3d social_force;

	// try to decelerate by incorporating a scaled `internal component` rotated by PI
	social_force = rotateVector(f_alpha_, IGN_PI);

	switch (level_) {

	case(FUZZY_LEVEL_LOW):
			social_force *= 0.20;
			break;
	case(FUZZY_LEVEL_MEDIUM):
			social_force *= 0.45;
			break;
	case(FUZZY_LEVEL_HIGH):
			social_force *= 0.75;
			break;
	case(FUZZY_LEVEL_EXTREME):
			social_force *= 1.00; // temporarily abandon going in the target direction
			break;

	}

	return (social_force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::rotateVector(const ignition::math::Vector3d &v, const double &angle) const {

	ignition::math::Vector3d rotated;
	rotated.X( v.X()*cos(angle) - v.Y()*sin(angle) );
	rotated.Y( v.X()*sin(angle) + v.Y()*cos(angle) );
	rotated.Z(0.0);

	return (rotated);

}

// ------------------------------------------------------------------- //

Defuzzifier::~Defuzzifier() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //


} /* namespace fuzz */
} /* namespace sfm */
