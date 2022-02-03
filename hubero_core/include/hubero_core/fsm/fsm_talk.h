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
		TALKING
	};

	FsmTalk(State state_init = State::MOVING_TO_GOAL): fsm(state_init), FsmEssentials("FsmTalk") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardMovingToGoal2Talking(const EventFsmTalk& event) const {
		return event.goal_reached;
	}

	bool guardTalking2MovingToGoal(const EventFsmTalk& event) const {
		return !event.goal_reached;
	}
	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerMovingToGoal2Talking(const EventFsmTalk& event) {
		logTransition("MOVING TO POSITION", "TALKING", event);
	}

	void transHandlerTalking2MovingToGoal(const EventFsmTalk& event) {
		logTransition("TALKING", "MOVING TO POSITION", event);
	}
	/** @} */ // end of transition handlers group

	/**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 */
	using transition_table = table<
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmTalk, State::TALKING, &FsmTalk::transHandlerMovingToGoal2Talking, &FsmTalk::guardMovingToGoal2Talking>,
		mem_fn_row<State::TALKING, EventFsmTalk, State::MOVING_TO_GOAL, &FsmTalk::transHandlerTalking2MovingToGoal, &FsmTalk::guardTalking2MovingToGoal>
	>;

	friend class fsmlite::fsm<FsmTalk>;
};

} // namespace hubero
