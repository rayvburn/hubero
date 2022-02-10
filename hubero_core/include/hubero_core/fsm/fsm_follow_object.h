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
		WAITING_FOR_MOVEMENT,
		FINISHED
	};

	FsmFollowObject(State state_init = State::FINISHED): fsm(state_init), FsmEssentials("FsmFollowObject") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardMoveToObject2Wait(const EventFsmFollowObject& event) const {
		return event.isObjectNearby() && event.isActive();
	}

	bool guardWait2MoveToObject(const EventFsmFollowObject& event) const {
		return !event.isObjectNearby() && event.isActive();
	}

	bool guardMoveToObject2Finished(const EventFsmFollowObject& event) const {
		return !event.isActive();
	}

	bool guardWait2Finished(const EventFsmFollowObject& event) const {
		return !event.isActive();
	}

	bool guardFinished2MoveToObject(const EventFsmFollowObject& event) const {
		return event.isActive();
	}

	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerMoveToObject2Wait(const EventFsmFollowObject& event) {
		logTransition("MOVING TO OBJECT", "WAITING FOR MOVEMENT", event);
		transitionHandler(State::MOVING_TO_GOAL, State::WAITING_FOR_MOVEMENT);
	}

	void transHandlerWait2MoveToObject(const EventFsmFollowObject& event) {
		logTransition("WAITING FOR MOVEMENT", "MOVING TO OBJECT", event);
		transitionHandler(State::WAITING_FOR_MOVEMENT, State::MOVING_TO_GOAL);
	}

	void transHandlerMoveToObject2Finished(const EventFsmFollowObject& event) {
		logTransition("MOVING TO OBJECT", "FINISHED", event);
		transitionHandler(State::MOVING_TO_GOAL, State::FINISHED);
	}

	void transHandlerWait2Finished(const EventFsmFollowObject& event) {
		logTransition("WAITING FOR MOVEMENT", "FINISHED", event);
		transitionHandler(State::WAITING_FOR_MOVEMENT, State::FINISHED);
	}

	void transHandlerFinished2MoveToObject(const EventFsmFollowObject& event) {
		logTransition("FINISHED", "MOVING TO OBJECT", event);
		transitionHandler(State::FINISHED, State::MOVING_TO_GOAL);
	}

	/** @} */ // end of transition handlers group

	/**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 */
	using transition_table = table<
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmFollowObject, State::WAITING_FOR_MOVEMENT, &FsmFollowObject::transHandlerMoveToObject2Wait, &FsmFollowObject::guardMoveToObject2Wait>,
		mem_fn_row<State::WAITING_FOR_MOVEMENT, EventFsmFollowObject, State::MOVING_TO_GOAL, &FsmFollowObject::transHandlerWait2MoveToObject, &FsmFollowObject::guardWait2MoveToObject>,
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmFollowObject, State::FINISHED, &FsmFollowObject::transHandlerMoveToObject2Finished, &FsmFollowObject::guardMoveToObject2Finished>,
		mem_fn_row<State::WAITING_FOR_MOVEMENT, EventFsmFollowObject, State::FINISHED, &FsmFollowObject::transHandlerWait2Finished, &FsmFollowObject::guardWait2Finished>,
		mem_fn_row<State::FINISHED, EventFsmFollowObject, State::MOVING_TO_GOAL, &FsmFollowObject::transHandlerFinished2MoveToObject, &FsmFollowObject::guardFinished2MoveToObject>
	>;

	friend class fsmlite::fsm<FsmFollowObject>;
};

} // namespace hubero
