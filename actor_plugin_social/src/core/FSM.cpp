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

FSM::FSM(): state_changed_flag_(true) {

#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS
	state_curr_.name = ACTOR_STATE_ALIGN_TARGET;
#ifdef FSM_VOID_PTR
	state_curr_.handler = nullptr;
#else
	//state_curr_.fn = 0;
#endif

#else
	addState(ACTOR_STATE_INITIAL);
	state_curr_ = ACTOR_STATE_INITIAL;
#endif

}

// ------------------------------------------------------------------- //

#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS

#ifdef FSM_VOID_PTR
void FSM::addState(const actor::ActorState &state, void (*handler)(gazebo::common::UpdateInfo)) {
#else

//void FSM::addState(const actor::ActorState &state, std::function<void (gazebo::common::UpdateInfo)> fn) {
void addState(const actor::ActorState &state, boost::function<void (gazebo::common::UpdateInfo)> fn) {
#endif

	bool state_already_added = false;
	std::tie(state_already_added, std::ignore) = isStateValid(state);

	if ( !state_already_added ) {
		actor::core::State new_state;
		new_state.name = state;
#ifdef FSM_VOID_PTR
		new_state.handler = handler;
#else
		new_state.fn = fn;
#endif
		states_.push_back(new_state);
	}

}
#else

void FSM::addState(const actor::ActorState &state) {

	bool state_already_added = false;
	std::tie(state_already_added, std::ignore) = isStateValid(state);

	if ( !state_already_added ) {
		states_.push_back(state);
	}

}

#endif

// ------------------------------------------------------------------- //

void FSM::setState(const actor::ActorState &new_state) {

	std::cout << "FSM: set State INTO!" << std::endl;
	if ( state_curr_ != new_state ) {

		bool is_valid = false;
		unsigned int index = 0;
		std::tie(is_valid, index) = isStateValid(new_state);

		if ( is_valid ) {
			std::cout << "FSM: state set is VALID!\t" << new_state << std::endl;
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

#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS
void FSM::executeCurrentState(const gazebo::common::UpdateInfo &info) {
	gazebo::common::UpdateInfo copy = info;
	std::cout << "FSM::executeCurrentState - fn address: " << &(state_curr_.fn) << "\t" << std::endl;
#ifdef FSM_VOID_PTR
	state_curr_.handler(copy);
#else
	state_curr_.fn(copy);
#endif
}
#endif

// ------------------------------------------------------------------- //

actor::ActorState FSM::getState() const {
#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS
	return (state_curr_.name);
#else
	return (state_curr_);
#endif
}

// ------------------------------------------------------------------- //

/// check if state with given name has already been added
std::tuple<bool, unsigned int> FSM::isStateValid(const actor::ActorState &new_state) {

	for ( unsigned int i = 0; i < states_.size(); i++ ) {

#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS
		if ( states_.at(i).name == new_state ) {
#else
		if ( states_.at(i) == new_state ) {
#endif
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
