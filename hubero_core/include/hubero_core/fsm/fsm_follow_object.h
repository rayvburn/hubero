#pragma once

#include <hubero_core/fsm/fsm_essentials.h>
#include <hubero_core/events/event_fsm_follow_object.h>

// fsmlite
#include <fsm.h>

namespace hubero {

class FsmFollowObject: public fsmlite::fsm<FsmFollowObject>, public FsmEssentials {
public:
	enum State {
		MOVING_TO_GOAL = 0,
		WAITING_FOR_MOVEMENT
	};

	FsmFollowObject(State state_init = State::MOVING_TO_GOAL): fsm(state_init), FsmEssentials("FsmFollowObject") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardMoveToObject2Wait(const EventFsmFollowObject& event) const {
		return event.object_nearby;
	}

	bool guardWait2MoveToObject(const EventFsmFollowObject& event) const {
		return !event.object_nearby;
	}
	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerMoveToObject2Wait(const EventFsmFollowObject& event) {
		logTransition("MOVING TO OBJECT", "WAITING FOR MOVEMENT", event);
	}

	void transHandlerWait2MoveToObject(const EventFsmFollowObject& event) {
		logTransition("WAITING FOR MOVEMENT", "MOVING TO OBJECT", event);
	}
	/** @} */ // end of transition handlers group

	/**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 */
	using transition_table = table<
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmFollowObject, State::WAITING_FOR_MOVEMENT, &FsmFollowObject::transHandlerMoveToObject2Wait, &FsmFollowObject::guardMoveToObject2Wait>,
		mem_fn_row<State::WAITING_FOR_MOVEMENT, EventFsmFollowObject, State::MOVING_TO_GOAL, &FsmFollowObject::transHandlerWait2MoveToObject, &FsmFollowObject::guardWait2MoveToObject>
	>;

	friend class fsmlite::fsm<FsmFollowObject>;
};

} // namespace hubero
