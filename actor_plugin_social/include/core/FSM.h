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

class FSM {

public:

	FSM();

	void addState(const actor::ActorState &state);
	void setState(const actor::ActorState &new_state);
	bool didStateChange();
	actor::ActorState getState() const;
	virtual ~FSM();

private:

	std::tuple<bool, unsigned int> isStateValid(const actor::ActorState &new_state);
	actor::ActorState state_curr_;
	bool state_changed_flag_;
	std::vector<actor::ActorState> states_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_CORE_FSM_H_ */
