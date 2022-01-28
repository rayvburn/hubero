#pragma once

#include <hubero_core/tasks/task_predicates.h>
#include <hubero_common/logger.h>

// fsmlite
#include <fsm.h>

#include <algorithm>
#include <string>

namespace hubero {

struct EventFsmLieDown: public TaskPredicates {
	bool goal_reached;
    bool lied_down;
    bool stood_up;

    std::string toString() const {
        return TaskPredicates::toString()
            + " goalReached: " + std::to_string(goal_reached)
            + " liedDown: " + std::to_string(lied_down)
            + " stoodUp: " + std::to_string(stood_up);
    }
};

class FsmLieDown: public fsmlite::fsm<FsmLieDown> {
public:
    enum State {
        MOVING_TO_GOAL = 0,
        LYING_DOWN,
        LYING,
        STANDING_UP,
        STANDING
    };

    FsmLieDown(State state_init = State::MOVING_TO_GOAL):
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
	void logTransition(const EventFsmLieDown& event) const {
        if (!logging_verbose_) {
		    return;
	    }
	    HUBERO_LOG("[%s].[FsmLieDown] transition conditions\r\n%s\r\n", logger_text_.c_str(), event.toString().c_str());
    }

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
        HUBERO_LOG("[%s].[FsmLieDown] transition from MOVING TO POSITION to LYING DOWN\r\n", logger_text_.c_str());
        logTransition(event);
    }

    void transHandlerLyingDown2Lying(const EventFsmLieDown& event) {
        HUBERO_LOG("[%s].[FsmLieDown] transition from LYING DOWN to LYING\r\n", logger_text_.c_str());
        logTransition(event);
    }

    void transHandlerLying2StandingUp(const EventFsmLieDown& event) {
        HUBERO_LOG("[%s].[FsmLieDown] transition from LYING to STANDING UP\r\n", logger_text_.c_str());
        logTransition(event);
    }

    void transHandlerStandingUp2Standing(const EventFsmLieDown& event) {
        HUBERO_LOG("[%s].[FsmLieDown] transition from STANDING UP to STANDING\r\n", logger_text_.c_str());
        logTransition(event);
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

	bool logging_verbose_;
	std::string logger_text_;

    friend class fsmlite::fsm<FsmLieDown>;
};

} // namespace hubero
