/*
 * Defuzzifier.h
 *
 *  Created on: Apr 20, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_DEFUZZIFIER_H_
#define INCLUDE_SFM_FUZZ_DEFUZZIFIER_H_

#include "sfm/fuzz/Fuzzifier.h"

namespace sfm {
namespace fuzz {

class Defuzzifier {

public:

	/// \brief Default constructor
	Defuzzifier();

	void setSocialConditionAndLevel(const sfm::fuzz::SocialCondition &condition, const sfm::fuzz::FuzzyLevel &level);

	/// \return A tuple with an additional internal
	/// force coefficient and rotated interaction
	/// force vector
	std::tuple<double, ignition::math::Vector3d> defuzzifyObjectRight(const double &fuzzy_f_alpha, const ignition::math::Vector3d &f_alpha_beta);

	/// \return A modified social force vector
	ignition::math::Vector3d defuzzifySfDirection(const ignition::math::Vector3d &social_force);

	/// \brief Default destructor
	virtual ~Defuzzifier();

private:

	/// \brief 2D rotation around Z-axis
	ignition::math::Vector3d rotateVector(const ignition::math::Vector3d &f_alpha_beta, const double &angle);
	sfm::fuzz::SocialCondition condition_;
	sfm::fuzz::FuzzyLevel level_;

};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_DEFUZZIFIER_H_ */
