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

Defuzzifier::Defuzzifier(): condition_(sfm::fuzz::Fuzzifier::SocialCondition::SFM_CONDITION_UNKNOWN),
		level_(sfm::fuzz::Fuzzifier::FuzzyLevel::FUZZY_LEVEL_UNKNOWN)
{ }

// ------------------------------------------------------------------- //

void Defuzzifier::setSocialConditionAndLevel(const sfm::fuzz::Fuzzifier::SocialCondition &condition, const sfm::fuzz::Fuzzifier::FuzzyLevel &level) {
	condition_ = condition;
	level_ = level;
}

// ------------------------------------------------------------------- //

Defuzzifier::~Defuzzifier() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
