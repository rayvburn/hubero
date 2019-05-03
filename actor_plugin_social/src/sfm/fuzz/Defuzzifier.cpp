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

Defuzzifier::Defuzzifier(): condition_(sfm::fuzz::SocialCondition::SFM_CONDITION_UNKNOWN),
		level_(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_UNKNOWN)
{ }

// ------------------------------------------------------------------- //

void Defuzzifier::setSocialConditionAndLevel(const sfm::fuzz::SocialCondition &condition, const sfm::fuzz::FuzzyLevel &level) {
	condition_ = condition;
	level_ = level;
}

// ------------------------------------------------------------------- //

std::tuple<double, ignition::math::Vector3d> Defuzzifier::defuzzifyObjectRight(const double &fuzzy_f_alpha, const ignition::math::Vector3d &f_alpha_beta) {

	if ( condition_ == sfm::fuzz::SocialCondition::SFM_CONDITION_UNKNOWN ||
		 level_ == sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_UNKNOWN ) {
		/* nothing has been detected by Fuzzifier - return
		 * non-changed vector and non-modifying coefficient (1.00) */
		return ( std::make_tuple(fuzzy_f_alpha, f_alpha_beta) );
	}

	double internal_force_coeff = fuzzy_f_alpha;
	ignition::math::Vector3d f_alpha_beta_rot( f_alpha_beta.X(), f_alpha_beta.Y(), 0.0 );

	switch ( condition_ ) {

	case(sfm::fuzz::SocialCondition::SFM_CONDITION_DYNAMIC_OBJECT_RIGHT):

			switch ( level_ ) {

			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_LOW):
					internal_force_coeff = 0.80;
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_MEDIUM):
					internal_force_coeff = 0.70;
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_HIGH):
					internal_force_coeff = 0.60;
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_EXTREME):
					internal_force_coeff = 0.50;
					break;

			} /* switch ( level_ ) */

			break; /* SFM_CONDITION_DYNAMIC_OBJECT_RIGHT */

	case(sfm::fuzz::SocialCondition::SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR):

			/* rotate vector 180 deg */
			f_alpha_beta_rot = rotateVector(f_alpha_beta, IGN_PI_2);

			switch ( level_ ) {

			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_LOW):
					internal_force_coeff = 0.75;
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_MEDIUM):
					internal_force_coeff = 0.60;
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_HIGH):
					internal_force_coeff = 0.45;
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_EXTREME):
					internal_force_coeff = 0.30;
					break;

			} /* switch ( level_ ) */

			break; /* SFM_CONDITION_DYNAMIC_OBJECT_RIGHT_MOVES_PERPENDICULAR */

	case(sfm::fuzz::SocialCondition::SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE):

			switch ( level_ ) {

			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_LOW):
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_MEDIUM):
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_HIGH):
					break;
			case(sfm::fuzz::FuzzyLevel::FUZZY_LEVEL_EXTREME):
					break;

			} /* switch ( level_ ) */

			break; /* SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE */

	}

	// leave the smallest coefficient
	if ( fuzzy_f_alpha < internal_force_coeff ) {
		internal_force_coeff = fuzzy_f_alpha;
	}

	return ( std::make_tuple(internal_force_coeff, f_alpha_beta_rot) );

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Defuzzifier::rotateVector(const ignition::math::Vector3d &f_alpha_beta, const double &angle) {

	ignition::math::Vector3d rotated;
	rotated.X( f_alpha_beta.X()*cos(angle) - f_alpha_beta.Y()*sin(angle) );
	rotated.Y( f_alpha_beta.X()*sin(angle) + f_alpha_beta.Y()*cos(angle) );
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
