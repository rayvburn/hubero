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

	FsmLieDown(State state_init = State::STANDING): fsm(state_init), FsmEssentials("FsmLieDown") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardMovingToGoal2LyingDown(const EventFsmLieDown& event) const {
		return event.isNavigationSucceeded();
	}

	bool guardLyingDown2Lying(const EventFsmLieDown& event) const {
		return event.hasLiedDown();
	}

	bool guardLying2StandingUp(const EventFsmLieDown& event) const {
		return event.isAborted();
	}

	bool guardStandingUp2Standing(const EventFsmLieDown& event) const {
		return event.hasStoodUp();
	}

	bool guardStanding2MovingToGoal(const EventFsmLieDown& event) const {
		return event.isActive() && !event.isNavigationActive();
	}
	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerMovingToGoal2LyingDown(const EventFsmLieDown& event) {
		logTransition("MOVING TO POSITION", "LYING DOWN", event);
		transitionHandler(State::MOVING_TO_GOAL, State::LYING_DOWN);
	}

	void transHandlerLyingDown2Lying(const EventFsmLieDown& event) {
		logTransition("LYING DOWN", "LYING", event);
		transitionHandler(State::LYING_DOWN, State::LYING);
	}

	void transHandlerLying2StandingUp(const EventFsmLieDown& event) {
		logTransition("LYING", "STANDING UP", event);
		transitionHandler(State::LYING, State::STANDING_UP);
	}

	void transHandlerStandingUp2Standing(const EventFsmLieDown& event) {
		logTransition("STANDING UP", "STANDING", event);
		transitionHandler(State::STANDING_UP, State::STANDING);
	}

	void transHandlerStanding2MovingToGoal(const EventFsmLieDown& event) {
		logTransition("STANDING", "MOVING TO POSITION", event);
		transitionHandler(State::STANDING, State::MOVING_TO_GOAL);
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
		mem_fn_row<State::STANDING_UP, EventFsmLieDown, State::STANDING, &FsmLieDown::transHandlerStandingUp2Standing, &FsmLieDown::guardStandingUp2Standing>,
		mem_fn_row<State::STANDING, EventFsmLieDown, State::MOVING_TO_GOAL, &FsmLieDown::transHandlerStanding2MovingToGoal, &FsmLieDown::guardStanding2MovingToGoal>
	>;

	friend class fsmlite::fsm<FsmLieDown>;
};

} // namespace hubero
