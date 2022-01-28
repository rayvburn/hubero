#pragma once

#include <hubero_core/tasks/task_predicates.h>
#include <hubero_common/logger.h>

// fsmlite
#include <fsm.h>

#include <algorithm>
#include <string>

namespace hubero {

struct EventFsmTalk: public TaskPredicates {
	bool goal_reached;

    std::string toString() const {
        return TaskPredicates::toString()
            + " goalReached: " + std::to_string(goal_reached);
    }
};

/**
 * @brief FSM for talking task
 * @details TALKING -> MOVING_TO_GOAL is allowed for TalkObject version of the task (object may move away)
 */
class FsmTalk: public fsmlite::fsm<FsmTalk> {
public:
    enum State {
        MOVING_TO_GOAL = 0,
        TALKING
    };

    FsmTalk(State state_init = State::MOVING_TO_GOAL):
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
	void logTransition(const EventFsmTalk& event) const {
        if (!logging_verbose_) {
		    return;
	    }
	    HUBERO_LOG("[%s].[FsmTalk] transition conditions\r\n%s\r\n", logger_text_.c_str(), event.toString().c_str());
    }

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
        HUBERO_LOG("[%s].[FsmTalk] transition from MOVING TO POSITION to TALKING\r\n", logger_text_.c_str());
        logTransition(event);
    }

    void transHandlerTalking2MovingToGoal(const EventFsmTalk& event) {
        HUBERO_LOG("[%s].[FsmTalk] transition from TALKING to MOVING TO POSITION\r\n", logger_text_.c_str());
        logTransition(event);
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

	bool logging_verbose_;
	std::string logger_text_;

    friend class fsmlite::fsm<FsmTalk>;
};

} // namespace hubero
