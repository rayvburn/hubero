/*
 * FSM.h
 *
 *  Created on: Apr 8, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_CORE_FSM_H_
#define INCLUDE_CORE_FSM_H_

#include "core/Enums.h"
#include <vector>
#include <tuple>

namespace actor {
namespace core {

// -------------------------
// TODO: store a pointer to a transition function

/// \brief A simple Finite State Machine class
/// which stores a current application state;
/// based on the state an Actor class sets
/// a transition function pointer
class FSM {

public:

	/// \brief Default constructor
	FSM();

	/// \brief Adds a new state
	void addState(const actor::ActorState &state);

	/// \brief Sets a current state
	void setState(const actor::ActorState &new_state);

	/// \brief Flag indicating whether a state
	/// has changed since last didStateChange()
	/// invocation
	bool didStateChange();

	/// \brief Sets state to the previous one. This can't be utilized to set
	/// pre-previous state, buffer stores only the current and the previous value.
	/// After restoring previous value next restoration try will not change state.
	void restorePreviousState();

	/// \brief Returns a current state of an FSM
	actor::ActorState getState() const;

	/// \brief Returns a previous state of an FSM
	actor::ActorState getStatePrevious() const;

	/// \brief Default destructor
	virtual ~FSM();

private:

	/// \brief Helper method which checks whether a given
	/// state is valid; it searches added states vector
	/// trying to find a given one
	std::tuple<bool, unsigned int> isStateValid(const actor::ActorState &new_state);

	/// \brief Stores a current state
	actor::ActorState state_curr_;

	/// \brief Stores a previous state
	actor::ActorState state_prev_;

	/// \brief Stores a flag indicating whether
	/// a state has changed
	bool state_changed_flag_;

	/// \brief Vector containing all previously
	/// added states with an addState() method
	std::vector<actor::ActorState> states_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_CORE_FSM_H_ */
