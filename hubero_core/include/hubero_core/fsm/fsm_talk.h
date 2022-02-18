#pragma once

#include <hubero_core/fsm/fsm_essentials.h>
#include <hubero_core/events/event_fsm_talk.h>

// fsmlite
#include <fsm.h>

namespace hubero {

/**
 * @brief FSM for talking task
 * @details TALKING -> MOVING_TO_GOAL is allowed for TalkObject version of the task (object may move away)
 */
class FsmTalk: public fsmlite::fsm<FsmTalk>, public FsmEssentials {
public:
	enum State {
		MOVING_TO_GOAL = 0,
		TALKING,
		FINISHED
	};

	FsmTalk(State state_init = State::FINISHED): fsm(state_init), FsmEssentials("FsmTalk") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardFinished2MovingToGoal(const EventFsmTalk& event) const {
		return event.isActive();
	}

	bool guardMovingToGoal2Finished(const EventFsmTalk& event) const {
		return !event.isActive();
	}

	bool guardMovingToGoal2Talking(const EventFsmTalk& event) const {
		return event.isNavigationSucceeded();
	}

	bool guardTalking2Finished(const EventFsmTalk& event) const {
		return !event.isActive();
	}

	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerFinished2MovingToGoal(const EventFsmTalk& event) {
		logTransition("FINISHED", "MOVING TO POSITION", event);
		transitionHandler(State::FINISHED, State::MOVING_TO_GOAL);
	}

	void transHandlerMovingToGoal2Finished(const EventFsmTalk& event) {
		logTransition("MOVING TO POSITION", "FINISHED", event);
		transitionHandler(State::MOVING_TO_GOAL, State::FINISHED);
	}

	void transHandlerMovingToGoal2Talking(const EventFsmTalk& event) {
		logTransition("MOVING TO POSITION", "TALKING", event);
		transitionHandler(State::MOVING_TO_GOAL, State::TALKING);
	}

	void transHandlerTalking2Finished(const EventFsmTalk& event) {
		logTransition("TALKING", "FINISHED", event);
		transitionHandler(State::TALKING, State::FINISHED);
	}

	/** @} */ // end of transition handlers group

	/**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 */
	using transition_table = table<
		mem_fn_row<State::FINISHED, EventFsmTalk, State::MOVING_TO_GOAL, &FsmTalk::transHandlerFinished2MovingToGoal, &FsmTalk::guardFinished2MovingToGoal>,
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmTalk, State::FINISHED, &FsmTalk::transHandlerMovingToGoal2Finished, &FsmTalk::guardMovingToGoal2Finished>,
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmTalk, State::TALKING, &FsmTalk::transHandlerMovingToGoal2Talking, &FsmTalk::guardMovingToGoal2Talking>,
		mem_fn_row<State::TALKING, EventFsmTalk, State::FINISHED, &FsmTalk::transHandlerTalking2Finished, &FsmTalk::guardTalking2Finished>
	>;

	friend class fsmlite::fsm<FsmTalk>;
};

} // namespace hubero
