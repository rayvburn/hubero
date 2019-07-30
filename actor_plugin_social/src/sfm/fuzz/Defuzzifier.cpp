/*
 * Defuzzifier.cpp
 *
 *  Created on: Apr 20, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/Defuzzifier.h>

namespace sfm {
namespace fuzz {

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
			} else if ( direction_ == FUZZ_DIR_OPPOSITE ) {
				// do nothing, default SFM formulation should take care of slight turning right
			} else if ( direction_ == FUZZ_DIR_PERP_OUT ) {

			} else if ( direction_ == FUZZ_DIR_PERP_CROSS ) {

			}

			break;

	case(FUZZ_LOCATION_LEFT):

			if ( direction_ == FUZZ_DIR_EQUAL ) {
				// do nothing
			} else if ( direction_ == FUZZ_DIR_OPPOSITE ) {
				// do nothing, default SFM formulation should take care of slight turning right
			} else if ( direction_ == FUZZ_DIR_PERP_OUT ) {

			} else if ( direction_ == FUZZ_DIR_PERP_CROSS ) {

			}

			break;

	case(FUZZ_LOCATION_BACK):

			if ( direction_ == FUZZ_DIR_EQUAL ) {
				// do nothing
			} else if ( direction_ == FUZZ_DIR_OPPOSITE ) {
				// do nothing, default SFM formulation should take care of slight turning right
			} else if ( direction_ == FUZZ_DIR_PERP_OUT ) {

			} else if ( direction_ == FUZZ_DIR_PERP_CROSS ) {

			}

			break;

	case(FUZZ_LOCATION_RIGHT):

			if ( direction_ == FUZZ_DIR_EQUAL ) {
				// do nothing
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
// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::passRight() const {

	ignition::math::Vector3d social_force;

	return (social_force);

}

// ------------------------------------------------------------------- //

// FIXME: passLeft for left side will probably be different
ignition::math::Vector3d Defuzzifier::passLeft() const {

	ignition::math::Vector3d social_force;

	// rotate f_alpha_beta perpendicular component by Pi
	social_force = rotateVector(f_alpha_beta_p_, IGN_PI);

	// rotated component must be multiplied (>1.00) to have an opposite effect
	// (relative to perpendicular component), otherwise it will just be `equalized`
	switch (level_) {

	case(FUZZY_LEVEL_LOW):
			social_force *= 1.20;
			break;
	case(FUZZY_LEVEL_MEDIUM):
			social_force *= 1.75;
			break;
	case(FUZZY_LEVEL_HIGH):
			social_force *= 2.25;
			break;
	case(FUZZY_LEVEL_EXTREME):
			social_force *= 3.00;
			break;

	}

	return (social_force);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::letPass() const {

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

// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //
// ------------------------------------------------------------------- //
//
//std::tuple<double, ignition::math::Vector3d> Defuzzifier::defuzzifyObjectRight(const double &fuzzy_f_alpha, const ignition::math::Vector3d &f_alpha_beta) {
//
//	if ( location_ == sfm::fuzz::FuzzLocation::SFM_CONDITION_UNKNOWN ||
//		 level_ == sfm::fuzz::FuzzLevel::FUZZY_LEVEL_UNKNOWN ) {
//		/* nothing has been detected by Fuzzifier - return
//		 * non-changed vector and non-modifying coefficient (1.00) */
//		return ( std::make_tuple(fuzzy_f_alpha, f_alpha_beta) );
//	}
//
//	double internal_force_coeff = fuzzy_f_alpha;
//	ignition::math::Vector3d f_alpha_beta_rot( f_alpha_beta.X(), f_alpha_beta.Y(), 0.0 );
//
//	switch ( location_ ) {
//
//	case(sfm::fuzz::FuzzLocation::SFM_CONDITION_DYNAMIC_OBJECT_RIGHT):
//
//			switch ( level_ ) {
//
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_LOW):
//					internal_force_coeff = 0.80;
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_MEDIUM):
//					internal_force_coeff = 0.70;
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_HIGH):
//					internal_force_coeff = 0.60;
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_EXTREME):
//					internal_force_coeff = 0.50;
//					break;
//
//			} /* switch ( level_ ) */
//
//			break; /* SFM_CONDITION_DYNAMIC_OBJECT_RIGHT */
//
//	case(sfm::fuzz::FuzzLocation::SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR):
//
//			/* rotate vector 180 deg */
//			f_alpha_beta_rot = rotateVector(f_alpha_beta, IGN_PI_2);
//
//			switch ( level_ ) {
//
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_LOW):
//					internal_force_coeff = 0.75;
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_MEDIUM):
//					internal_force_coeff = 0.60;
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_HIGH):
//					internal_force_coeff = 0.45;
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_EXTREME):
//					internal_force_coeff = 0.30;
//					break;
//
//			} /* switch ( level_ ) */
//
//			break; /* SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR */
//
//	case(sfm::fuzz::FuzzLocation::SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE):
//
//			switch ( level_ ) {
//
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_LOW):
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_MEDIUM):
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_HIGH):
//					break;
//			case(sfm::fuzz::FuzzLevel::FUZZY_LEVEL_EXTREME):
//					break;
//
//			} /* switch ( level_ ) */
//
//			break; /* SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE */
//
//	}
//
//	// leave the smallest coefficient
//	if ( fuzzy_f_alpha < internal_force_coeff ) {
//		internal_force_coeff = fuzzy_f_alpha;
//	}
//
//	return ( std::make_tuple(internal_force_coeff, f_alpha_beta_rot) );
//
//}


} /* namespace fuzz */
} /* namespace sfm */
