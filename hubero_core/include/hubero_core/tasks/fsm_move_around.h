#pragma once

#include <hubero_core/tasks/task_predicates.h>
#include <hubero_common/logger.h>

// fsmlite
#include <fsm.h>

#include <algorithm>
#include <string>

namespace hubero {

struct EventFsmMoveAround: public TaskPredicates {
	bool goal_reached;
    bool goal_selected;

    std::string toString() const {
        return TaskPredicates::toString()
            + " goalReached: " + std::to_string(goal_reached)
            + " goalSelected: " + std::to_string(goal_selected);
    }
};

class FsmMoveAround: public fsmlite::fsm<FsmMoveAround> {
public:
    enum State {
        MOVING_TO_GOAL = 0,
        CHOOSING_GOAL
    };

    FsmMoveAround(State state_init = State::MOVING_TO_GOAL):
        fsm(state_init),
        logging_verbose_(false) {}

    void setLoggingVerbosity(bool enable_verbose) {
        logging_verbose_ = enable_verbose;
    }

	void setLoggerPreamble(const std::string& text) {
        logger_text_ = text;
    }

protected:
    /**
	 * @brief Logs @ref event details if @ref logging_verbose_ was set to true
	 */
	void logTransition(const EventFsmMoveAround& event) const {
        if (!logging_verbose_) {
		    return;
	    }
	    HUBERO_LOG("[%s].[FsmMoveAround] transition conditions\r\n%s\r\n", logger_text_.c_str(), event.toString().c_str());
    }

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
        HUBERO_LOG("[%s].[FsmMoveAround] transition from MOVING TO POSITION to CHOOSING GOAL\r\n", logger_text_.c_str());
        logTransition(event);
    }

    void transHandlerChoosingGoal2MovingToGoal(const EventFsmMoveAround& event) {
        HUBERO_LOG("[%s].[FsmMoveAround] transition from CHOOSING GOAL to MOVING TO POSITION\r\n", logger_text_.c_str());
        logTransition(event);
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

	bool logging_verbose_;
	std::string logger_text_;

    friend class fsmlite::fsm<FsmMoveAround>;
};

} // namespace hubero
