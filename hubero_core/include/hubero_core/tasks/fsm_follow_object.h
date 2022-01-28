#pragma once

#include <hubero_core/tasks/task_predicates.h>
#include <hubero_common/logger.h>

// fsmlite
#include <fsm.h>

#include <algorithm>
#include <string>

namespace hubero {

struct EventFsmFollowObject: public TaskPredicates {
	bool object_nearby;

    std::string toString() const {
        return TaskPredicates::toString()
            + " objectNearby: " + std::to_string(object_nearby);
    }
};

class FsmFollowObject: public fsmlite::fsm<FsmFollowObject> {
public:
    enum State {
        MOVING_TO_GOAL = 0,
        WAITING_FOR_MOVEMENT
    };

    FsmFollowObject(State state_init = State::MOVING_TO_GOAL):
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
	void logTransition(const EventFsmFollowObject& event) const {
        if (!logging_verbose_) {
		    return;
	    }
	    HUBERO_LOG("[%s].[FsmFollowObject] transition conditions\r\n%s\r\n", logger_text_.c_str(), event.toString().c_str());
    }

    /**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
    bool guardMoveToObject2Wait(const EventFsmFollowObject& event) const {
        return event.object_nearby;
    }

    bool guardWait2MoveToObject(const EventFsmFollowObject& event) const {
        return !event.object_nearby;
    }
    /** @} */ // end of guards group

    /**
	 * @defgroup Transition handlers
	 * @{
	 */
    void transHandlerMoveToObject2Wait(const EventFsmFollowObject& event) {
        HUBERO_LOG("[%s].[FsmFollowObject] transition from MOVING TO OBJECT to WAITING FOR MOVEMENT\r\n", logger_text_.c_str());
        logTransition(event);
    }

    void transHandlerWait2MoveToObject(const EventFsmFollowObject& event) {
        HUBERO_LOG("[%s].[FsmFollowObject] transition from WAITING FOR MOVEMENT to MOVING TO OBJECT\r\n", logger_text_.c_str());
        logTransition(event);
    }
    /** @} */ // end of transition handlers group

    /**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 */
	using transition_table = table<
        mem_fn_row<State::MOVING_TO_GOAL, EventFsmFollowObject, State::WAITING_FOR_MOVEMENT, &FsmFollowObject::transHandlerMoveToObject2Wait, &FsmFollowObject::guardMoveToObject2Wait>,
        mem_fn_row<State::WAITING_FOR_MOVEMENT, EventFsmFollowObject, State::MOVING_TO_GOAL, &FsmFollowObject::transHandlerWait2MoveToObject, &FsmFollowObject::guardWait2MoveToObject>
    >;

	bool logging_verbose_;
	std::string logger_text_;

    friend class fsmlite::fsm<FsmFollowObject>;
};

} // namespace hubero
