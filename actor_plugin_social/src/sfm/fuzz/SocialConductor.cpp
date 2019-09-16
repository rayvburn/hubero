/*
 * SocialConductor.cpp
 *
 *  Created on: Aug 6, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/SocialConductor.h>
#include <sfm/fuzz/Regions.h>

namespace sfm {
namespace fuzz {

// ------------------------------------------------------------------- //

// FIXME: parametrization
SocialConductor::SocialConductor(): SocialBehavioursDb(500.0) { }

// ------------------------------------------------------------------- //

void SocialConductor::apply(const double &fuzz_output) {

	// TODO: choose behaviour based on name

	// check whether the fuzzy logic block output is in a valid range
	if ( fuzz_output >  (static_cast<double>(FUZZ_BEH_TURN_LEFT) - 1.0) &&
		 fuzz_output <= (static_cast<double>(FUZZ_BEH_TURN_LEFT)) )
	{
		// apply `turn left` behavior
		sf_vector_.push_back(this->turnLeft());

	} else if ( fuzz_output >  (static_cast<double>(FUZZ_BEH_TURN_LEFT_ACCELERATE) - 1.0) &&
				fuzz_output <= (static_cast<double>(FUZZ_BEH_TURN_LEFT_ACCELERATE)) )
	{
		// apply `turn left and accelerate` behavior
		sf_vector_.push_back(this->turnLeftAccelerate());

	} else if ( fuzz_output >  (static_cast<double>(FUZZ_BEH_GO_ALONG) - 1.0) &&
			 	fuzz_output <= (static_cast<double>(FUZZ_BEH_GO_ALONG)) )
	{
		// apply `go along` behavior
		sf_vector_.push_back(this->goAlong());

	} else if ( fuzz_output >  (static_cast<double>(FUZZ_BEH_ACCELERATE) - 1.0) &&
		 		fuzz_output <= (static_cast<double>(FUZZ_BEH_ACCELERATE)) )
	{
		// apply `accelerate` behavior
		sf_vector_.push_back(this->accelerate());

	} else if ( fuzz_output >  (static_cast<double>(FUZZ_BEH_TURN_RIGHT_ACCELERATE) - 1.0) &&
	 			fuzz_output <= (static_cast<double>(FUZZ_BEH_TURN_RIGHT_ACCELERATE)) )
	{
		// apply `turn right and accelerate` behavior
		sf_vector_.push_back(this->turnRightAccelerate());

	} else if ( fuzz_output > (static_cast<double>(FUZZ_BEH_TURN_RIGHT) - 1.0) &&
 				fuzz_output <= (static_cast<double>(FUZZ_BEH_TURN_RIGHT)) )
	{
		// apply `turn right` behavior
		sf_vector_.push_back(this->turnRight());

	} else if ( fuzz_output >  (static_cast<double>(FUZZ_BEH_TURN_RIGHT_DECELERATE) - 1.0) &&
				fuzz_output <= (static_cast<double>(FUZZ_BEH_TURN_RIGHT_DECELERATE)) )
	{
		// apply `turn right and decelerate` behavior
		sf_vector_.push_back(this->turnRightDecelerate());

	} else if ( fuzz_output >  (static_cast<double>(FUZZ_BEH_STOP) - 1.0) &&
				fuzz_output <= (static_cast<double>(FUZZ_BEH_STOP)) )
	{
		// apply `stop` behavior
		sf_vector_.push_back(this->stop());

	} else if ( fuzz_output >  (static_cast<double>(FUZZ_BEH_DECELERATE) - 1.0) &&
				fuzz_output <= (static_cast<double>(FUZZ_BEH_DECELERATE)) )
	{
		// apply `decelerate` behavior
		sf_vector_.push_back(this->decelerate());

	} else {

		// prevents calling `superpose` when `fuzz_output` is invalid
		return;

	}
	superpose();

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialConductor::getSocialVector() const {
	return (sf_result_);
}

// ------------------------------------------------------------------- //

void SocialConductor::reset() {
	sf_vector_.clear();
	sf_result_ = ignition::math::Vector3d();
}

// ------------------------------------------------------------------- //

SocialConductor::~SocialConductor() { }

// ------------------------------------------------------------------- //

void SocialConductor::superpose() {

	// TODO: add crowd support - actual superposition
	// NOW: average value of vectors
	ignition::math::Vector3d avg;
	for ( size_t i = 0; i < sf_vector_.size(); i++ ) {
		avg += sf_vector_.at(i);
	}
	avg /= sf_vector_.size();

	// assign averaged vector to the resultative one
	sf_result_ = avg;

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
