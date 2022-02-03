#pragma once

#include <hubero_core/fsm/fsm_essentials.h>
#include <hubero_core/events/event_fsm_lie_down.h>

// fsmlite
#include <fsm.h>

namespace hubero {

class FsmLieDown: public fsmlite::fsm<FsmLieDown>, public FsmEssentials {
public:
	enum State {
		MOVING_TO_GOAL = 0,
		LYING_DOWN,
		LYING,
		STANDING_UP,
		STANDING
	};

	FsmLieDown(State state_init = State::MOVING_TO_GOAL): fsm(state_init), FsmEssentials("FsmLieDown") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardMovingToGoal2LyingDown(const EventFsmLieDown& event) const {
		return event.goal_reached;
	}

	bool guardLyingDown2Lying(const EventFsmLieDown& event) const {
		return event.lied_down;
	}

	bool guardLying2StandingUp(const EventFsmLieDown& event) const {
		return event.aborted;
	}

	bool guardStandingUp2Standing(const EventFsmLieDown& event) const {
		return event.stood_up;
	}

	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerMovingToGoal2LyingDown(const EventFsmLieDown& event) {
		logTransition("MOVING TO POSITION", "LYING DOWN", event);
	}

	void transHandlerLyingDown2Lying(const EventFsmLieDown& event) {
		logTransition("LYING DOWN", "LYING", event);
	}

	void transHandlerLying2StandingUp(const EventFsmLieDown& event) {
		logTransition("LYING", "STANDING UP", event);
	}

	void transHandlerStandingUp2Standing(const EventFsmLieDown& event) {
		logTransition("STANDING UP", "STANDING", event);
	}
	/** @} */ // end of transition handlers group

	/**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 */
	using transition_table = table<
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmLieDown, State::LYING_DOWN, &FsmLieDown::transHandlerMovingToGoal2LyingDown, &FsmLieDown::guardMovingToGoal2LyingDown>,
		mem_fn_row<State::LYING_DOWN, EventFsmLieDown, State::LYING, &FsmLieDown::transHandlerLyingDown2Lying, &FsmLieDown::guardLyingDown2Lying>,
		mem_fn_row<State::LYING, EventFsmLieDown, State::STANDING_UP, &FsmLieDown::transHandlerLying2StandingUp, &FsmLieDown::guardLying2StandingUp>,
		mem_fn_row<State::STANDING_UP, EventFsmLieDown, State::STANDING, &FsmLieDown::transHandlerStandingUp2Standing, &FsmLieDown::guardStandingUp2Standing>
	>;

	friend class fsmlite::fsm<FsmLieDown>;
};

} // namespace hubero
