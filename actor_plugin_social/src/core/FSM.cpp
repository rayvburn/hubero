/*
 * FSM.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: rayvburn
 */

#include <core/FSM.h>
#include <iostream> // debugging only

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

FSM::FSM(): state_changed_flag_(true) {

	addState(ACTOR_STATE_INITIAL);
	state_curr_ = ACTOR_STATE_INITIAL;
	state_prev_ = ACTOR_STATE_INITIAL;

}

// ------------------------------------------------------------------- //

void FSM::addState(const actor::ActorState &state) {

	bool state_already_added = false;
	std::tie(state_already_added, std::ignore) = isStateValid(state);

	if ( !state_already_added ) {
		states_.push_back(state);
	}

}

// ------------------------------------------------------------------- //

void FSM::setState(const actor::ActorState &new_state) {

	std::cout << "FSM: set State INTO!" << std::endl;
	if ( state_curr_ != new_state ) {

		bool is_valid = false;
		unsigned int index = 0;
		std::tie(is_valid, index) = isStateValid(new_state);

		if ( is_valid ) {
			std::cout << "FSM: state set is VALID!\t" << new_state << std::endl;
			state_prev_ = state_curr_;
			state_curr_ = states_.at(index);
			state_changed_flag_ = true;
		}
		return;

	}
	std::cout << "FSM: state set is INVALID!" << std::endl;

}

// ------------------------------------------------------------------- //

bool FSM::didStateChange() {
	bool flag = state_changed_flag_;
	state_changed_flag_ = false;
	return (flag);
}

// ------------------------------------------------------------------- //

void FSM::restorePreviousState() {
	state_curr_ = state_prev_;
}

// ------------------------------------------------------------------- //

actor::ActorState FSM::getState() const {
	return (state_curr_);
}

// ------------------------------------------------------------------- //

actor::ActorState FSM::getStatePrevious() const {
	return (state_prev_);
}

// ------------------------------------------------------------------- //

/// check if state with given name has already been added
std::tuple<bool, unsigned int> FSM::isStateValid(const actor::ActorState &new_state) {

	for ( unsigned int i = 0; i < states_.size(); i++ ) {
		if ( states_.at(i) == new_state ) {
			return ( std::make_tuple(true, i) );
		}
	}
	return ( std::make_tuple(false, 0) );

}

// ------------------------------------------------------------------- //

FSM::~FSM() { }

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
