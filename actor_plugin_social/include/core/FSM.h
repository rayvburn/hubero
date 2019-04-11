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
//#include <functional>	// std::function
#include <boost/function.hpp>

//#define FSM_VOID_PTR
#define FSM_DO_NOT_WRAP_ACTOR_METHODS

namespace actor {
namespace core {

// -------------------------

#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS
typedef struct {
	actor::ActorState name;
#ifdef FSM_VOID_PTR
	void (*handler)(gazebo::common::UpdateInfo);
#else
	//std::function<void (gazebo::common::UpdateInfo)> fn;
	boost::function<void (gazebo::common::UpdateInfo)> fn;
#endif
} State;
#endif

// -------------------------

class FSM {

public:

	FSM();

#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS
#ifdef FSM_VOID_PTR
	void addState(const actor::ActorState &state, void (*handler)(gazebo::common::UpdateInfo));
#else
	//void addState(const actor::ActorState &state, std::function<void (gazebo::common::UpdateInfo)> fn);
	void addState(const actor::ActorState &state, boost::function<void (gazebo::common::UpdateInfo)> fn);
#endif
#else
	void addState(const actor::ActorState &state);
#endif

	void setState(const actor::ActorState &new_state);
	bool didStateChange();
#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS
	void executeCurrentState(const gazebo::common::UpdateInfo &info);
#endif
	actor::ActorState getState() const;
	virtual ~FSM();

private:

	std::tuple<bool, unsigned int> isStateValid(const actor::ActorState &new_state);

#ifndef FSM_DO_NOT_WRAP_ACTOR_METHODS
	actor::core::State state_curr_;
	bool state_changed_flag_;
	std::vector<actor::core::State> states_;
#else
	actor::ActorState state_curr_;
	bool state_changed_flag_;
	std::vector<actor::ActorState> states_;
#endif

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_CORE_FSM_H_ */
