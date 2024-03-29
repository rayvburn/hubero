#pragma once

#include <hubero_core/fsm/fsm_essentials.h>
#include <hubero_core/events/event_fsm_sit_down.h>

// fsmlite
#include <fsm.h>

namespace hubero {

/**
 * @note This one is very similar to FsmLieDown
 * // TODO: try to unify these 2 FSMs
 */
class FsmSitDown: public fsmlite::fsm<FsmSitDown>, public FsmEssentials {
public:
	enum State {
		MOVING_TO_GOAL = 0,
		SITTING_DOWN,
		SITTING,
		STANDING_UP,
		STANDING
	};

	FsmSitDown(State state_init = State::STANDING): fsm(state_init), FsmEssentials("FsmSitDown") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardMovingToGoal2SittingDown(const EventFsmSitDown& event) const {
		return event.isNavigationSucceeded();
	}

	bool guardSittingDown2Sitting(const EventFsmSitDown& event) const {
		return event.hasSatDown();
	}

	bool guardSitting2StandingUp(const EventFsmSitDown& event) const {
		return event.isAborted();
	}

	bool guardStandingUp2Standing(const EventFsmSitDown& event) const {
		return event.hasStoodUp();
	}

	bool guardStanding2MovingToGoal(const EventFsmSitDown& event) const {
		return event.isActive() && !event.isNavigationActive();
	}

	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerMovingToGoal2SittingDown(const EventFsmSitDown& event) {
		logTransition("MOVING TO POSITION", "SITTING DOWN", event);
		transitionHandler(State::MOVING_TO_GOAL, State::SITTING_DOWN);
	}

	void transHandlerSittingDown2Sitting(const EventFsmSitDown& event) {
		logTransition("SITTING DOWN", "SITTING", event);
		transitionHandler(State::SITTING_DOWN, State::SITTING);
	}

	void transHandlerSitting2StandingUp(const EventFsmSitDown& event) {
		logTransition("SITTING", "STANDING UP", event);
		transitionHandler(State::SITTING, State::STANDING_UP);
	}

	void transHandlerStandingUp2Standing(const EventFsmSitDown& event) {
		logTransition("STANDING UP", "STANDING", event);
		transitionHandler(State::STANDING_UP, State::STANDING);
	}

	void transHandlerStanding2MovingToGoal(const EventFsmSitDown& event) {
		logTransition("STANDING", "MOVING TO POSITION", event);
		transitionHandler(State::STANDING, State::MOVING_TO_GOAL);
	}
	/** @} */ // end of transition handlers group

	/**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 */
	using transition_table = table<
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmSitDown, State::SITTING_DOWN, &FsmSitDown::transHandlerMovingToGoal2SittingDown, &FsmSitDown::guardMovingToGoal2SittingDown>,
		mem_fn_row<State::SITTING_DOWN, EventFsmSitDown, State::SITTING, &FsmSitDown::transHandlerSittingDown2Sitting, &FsmSitDown::guardSittingDown2Sitting>,
		mem_fn_row<State::SITTING, EventFsmSitDown, State::STANDING_UP, &FsmSitDown::transHandlerSitting2StandingUp, &FsmSitDown::guardSitting2StandingUp>,
		mem_fn_row<State::STANDING_UP, EventFsmSitDown, State::STANDING, &FsmSitDown::transHandlerStandingUp2Standing, &FsmSitDown::guardStandingUp2Standing>,
		mem_fn_row<State::STANDING, EventFsmSitDown, State::MOVING_TO_GOAL, &FsmSitDown::transHandlerStanding2MovingToGoal, &FsmSitDown::guardStanding2MovingToGoal>
	>;

	friend class fsmlite::fsm<FsmSitDown>;
};

} // namespace hubero
