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

	FsmSitDown(State state_init = State::MOVING_TO_GOAL): fsm(state_init), FsmEssentials("FsmSitDown") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardMovingToGoal2SittingDown(const EventFsmSitDown& event) const {
		return event.goal_reached;
	}

	bool guardSittingDown2Sitting(const EventFsmSitDown& event) const {
		return event.sat_down;
	}

	bool guardSitting2StandingUp(const EventFsmSitDown& event) const {
		return event.aborted;
	}

	bool guardStandingUp2Standing(const EventFsmSitDown& event) const {
		return event.stood_up;
	}

	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerMovingToGoal2SittingDown(const EventFsmSitDown& event) {
		logTransition("MOVING TO POSITION", "SITTING DOWN", event);
	}

	void transHandlerSittingDown2Sitting(const EventFsmSitDown& event) {
		logTransition("SITTING DOWN", "SITTING", event);
	}

	void transHandlerSitting2StandingUp(const EventFsmSitDown& event) {
		logTransition("SITTING", "STANDING UP", event);
	}

	void transHandlerStandingUp2Standing(const EventFsmSitDown& event) {
		logTransition("STANDING UP", "STANDING", event);
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
		mem_fn_row<State::STANDING_UP, EventFsmSitDown, State::STANDING, &FsmSitDown::transHandlerStandingUp2Standing, &FsmSitDown::guardStandingUp2Standing>
	>;

	friend class fsmlite::fsm<FsmSitDown>;
};

} // namespace hubero
