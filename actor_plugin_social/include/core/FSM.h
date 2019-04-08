/*
 * FSM.h
 *
 *  Created on: Apr 8, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_CORE_FSM_H_
#define INCLUDE_CORE_FSM_H_

#include "core/Enums.h"
#include "gazebo-8/gazebo/common/UpdateInfo.hh"
#include <vector>
#include <tuple>

namespace actor {
namespace core {

// -------------------------

typedef struct {
	actor::ActorState name;
	void (*handler)(const gazebo::common::UpdateInfo&);
} State;

// -------------------------

class FSM {

public:

	FSM();
	void addState(const actor::ActorState &state, void (*handler)(const gazebo::common::UpdateInfo&));
	void setState(const actor::ActorState &new_state);
	void executeCurrentState(const gazebo::common::UpdateInfo &info);
	actor::ActorState getState() const;
	virtual ~FSM();

private:

	std::tuple<bool, unsigned int> isStateValid(const actor::ActorState &new_state);
	actor::core::State state_curr_;
	std::vector<actor::core::State> states_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_CORE_FSM_H_ */
