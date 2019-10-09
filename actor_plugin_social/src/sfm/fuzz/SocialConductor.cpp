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

	/* 1) Behaviour selection based on output value */

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

void SocialConductor::apply(const std::string &term_name) {

	/* 2) Behaviour selection based on the highest membership term name */

	if ( term_name == "turn_left" ) {

		// apply `turn left` behavior
		sf_vector_.push_back(this->turnLeft());
		behaviour_active_ = FUZZ_BEH_TURN_LEFT;
		updateActiveBehaviour("turn_left"); // create a list of activated behaviours

	} else if ( term_name == "turn_left_accelerate" ) {

		// apply `turn left and accelerate` behavior
		sf_vector_.push_back(this->turnLeftAccelerate());
		behaviour_active_ = FUZZ_BEH_TURN_LEFT_ACCELERATE;
		updateActiveBehaviour("turn_left_accelerate");

	} else if ( term_name == "accelerate" ) {

		// apply `accelerate` behavior
		sf_vector_.push_back(this->accelerate());
		behaviour_active_ = FUZZ_BEH_ACCELERATE;
		updateActiveBehaviour("accelerate");

	} else if ( term_name == "go_along" ) {

		// apply `go along` behavior
		sf_vector_.push_back(this->goAlong());
		behaviour_active_ = FUZZ_BEH_GO_ALONG;
		updateActiveBehaviour("go_along");

	} else if ( term_name == "decelerate" ) {

		// apply `decelerate` behavior
		sf_vector_.push_back(this->decelerate());
		behaviour_active_ = FUZZ_BEH_DECELERATE;
		updateActiveBehaviour("decelerate");

	} else if ( term_name == "stop" ) {

		// apply `stop` behavior
		sf_vector_.push_back(this->stop());
		behaviour_active_ = FUZZ_BEH_STOP;
		updateActiveBehaviour("stop");

	} else if ( term_name == "turn_right_decelerate" ) {

		// apply `turn right and decelerate` behavior
		sf_vector_.push_back(this->turnRightDecelerate());
		behaviour_active_ = FUZZ_BEH_TURN_RIGHT_DECELERATE;
		updateActiveBehaviour("turn_right_decelerate");

	} else if ( term_name == "turn_right" ) {

		// apply `turn right` behavior
		sf_vector_.push_back(this->turnRight());
		behaviour_active_ = FUZZ_BEH_TURN_RIGHT;
		updateActiveBehaviour("turn_right");

	} else if ( term_name == "turn_right_accelerate" ) {

		// apply `turn right and accelerate` behavior
		sf_vector_.push_back(this->turnRightAccelerate());
		behaviour_active_ = FUZZ_BEH_TURN_RIGHT_ACCELERATE;
		updateActiveBehaviour("turn_right_accelerate");

	} else {

		// prevents calling `superpose` when [input] is invalid
		behaviour_active_ = FUZZ_BEH_NONE;
		// do not update the string to prevent having `none`
		// among valid behaviours
		return;

	}

	superpose();

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialConductor::getSocialVector() const {
	return (sf_result_);
}

// ------------------------------------------------------------------- //

uint8_t SocialConductor::getBehaviourActiveNum() const {
	return (behaviour_active_);
}

// ------------------------------------------------------------------- //

std::string SocialConductor::getBehaviourActive() const {

	// NOTE: rViz goes mad when tries to publish
	// an empty string (application crashes)
	if ( behaviour_active_str_.empty() ) {
		return ("none");
	}
	return (behaviour_active_str_);

}

// ------------------------------------------------------------------- //

void SocialConductor::reset() {
	sf_vector_.clear();
	sf_result_ = ignition::math::Vector3d();
	behaviour_active_ = 0;
	behaviour_active_str_.clear();
}

// ------------------------------------------------------------------- //

SocialConductor::~SocialConductor() { }

// ------------------------------------------------------------------- //

void SocialConductor::superpose() {

	// save a max length of a component to know
	// if there are few meaningful (i.e. non-zero length-ed)
	// elements in the vector but apparently their sum
	// is near-zero;
	// used to determine which behaviours are actually activated
	double max_len = 0.0;

	// TODO: add crowd support - actual superposition
	// NOW: average value of vectors
	ignition::math::Vector3d avg;
	for ( size_t i = 0; i < sf_vector_.size(); i++ ) {
		avg += sf_vector_.at(i);
		double len = sf_vector_.at(i).Length();
		if ( len > max_len ) { max_len = len; }
	}
	avg /= sf_vector_.size();

	// assign averaged vector to the resultative one
	sf_result_ = avg;

	// force `none` behaviour when `max len` is very small
	if ( max_len < 1e-03 ) {
		behaviour_active_ = FUZZ_BEH_NONE;
		behaviour_active_str_.clear();
	}

}

// ------------------------------------------------------------------- //

void SocialConductor::updateActiveBehaviour(const std::string &beh_name) {

	if ( behaviour_active_str_.empty() ) {
		behaviour_active_str_ = beh_name;
	} else {
		behaviour_active_str_.append("\n");
		behaviour_active_str_.append(beh_name);
	}

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
