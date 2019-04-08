/*
 * FSM.cpp
 *
 *  Created on: Apr 8, 2019
 *      Author: rayvburn
 */

#include <core/FSM.h>

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

FSM::FSM() {
	state_curr_.name = ACTOR_STATE_ALIGN_TARGET;
	state_curr_.handler = nullptr;
}

// ------------------------------------------------------------------- //

void FSM::addState(const actor::ActorState &state, void (*handler)(const gazebo::common::UpdateInfo&)) {


	bool state_already_added = false;
	std::tie(state_already_added, std::ignore) = isStateValid(state);

	if ( !state_already_added ) {
		actor::core::State new_state;
		new_state.name = state;
		new_state.handler = handler;
		states_.push_back(new_state);
	}

}

// ------------------------------------------------------------------- //

void FSM::setState(const actor::ActorState &new_state) {

	bool is_valid = false;
	unsigned int index = 0;
	std::tie(is_valid, index) = isStateValid(new_state);

	if ( is_valid ) {
		state_curr_ = states_.at(index);
	}

}

// ------------------------------------------------------------------- //

void FSM::executeCurrentState(const gazebo::common::UpdateInfo &info) {
	state_curr_.handler(info);
}

// ------------------------------------------------------------------- //

actor::ActorState FSM::getState() const {
	return (state_curr_.name);
}

// ------------------------------------------------------------------- //

/// check if state with given name has already been added
std::tuple<bool, unsigned int> FSM::isStateValid(const actor::ActorState &new_state) {

	for ( unsigned int i = 0; i < states_.size(); i++ ) {
		if ( states_.at(i).name == new_state ) {
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
