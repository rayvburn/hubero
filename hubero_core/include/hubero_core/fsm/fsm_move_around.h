#pragma once

#include <hubero_core/fsm/fsm_essentials.h>
#include <hubero_core/events/event_fsm_move_around.h>

// fsmlite
#include <fsm.h>

namespace hubero {

class FsmMoveAround: public fsmlite::fsm<FsmMoveAround>, public FsmEssentials {
public:
	enum State {
		MOVING_TO_GOAL = 0,
		CHOOSING_GOAL
	};

	FsmMoveAround(State state_init = State::CHOOSING_GOAL): fsm(state_init), FsmEssentials("FsmMoveAround") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardMovingToGoal2ChoosingGoal(const EventFsmMoveAround& event) const {
		return event.goal_reached && !event.goal_selected;
	}

	bool guardChoosingGoal2MovingToGoal(const EventFsmMoveAround& event) const {
		return !event.goal_reached && event.goal_selected;
	}
	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerMovingToGoal2ChoosingGoal(const EventFsmMoveAround& event) {
		logTransition("MOVING TO POSITION", "CHOOSING GOAL", event);
	}

	void transHandlerChoosingGoal2MovingToGoal(const EventFsmMoveAround& event) {
		logTransition("CHOOSING GOAL", "MOVING TO POSITION", event);
	}
	/** @} */ // end of transition handlers group

	/**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 */
	using transition_table = table<
		mem_fn_row<State::MOVING_TO_GOAL, EventFsmMoveAround, State::CHOOSING_GOAL, &FsmMoveAround::transHandlerMovingToGoal2ChoosingGoal, &FsmMoveAround::guardMovingToGoal2ChoosingGoal>,
		mem_fn_row<State::CHOOSING_GOAL, EventFsmMoveAround, State::MOVING_TO_GOAL, &FsmMoveAround::transHandlerChoosingGoal2MovingToGoal, &FsmMoveAround::guardChoosingGoal2MovingToGoal>
	>;

	friend class fsmlite::fsm<FsmMoveAround>;
};

} // namespace hubero
