#pragma once

#include <hubero_core/tasks/task_predicates.h>
#include <hubero_common/logger.h>

// fsmlite
#include <fsm.h>

#include <algorithm>
#include <string>

namespace hubero {


struct EventFsmSitDown: public TaskPredicates {
	bool goal_reached;
	bool sat_down;
	bool stood_up;

	std::string toString() const {
		return TaskPredicates::toString()
			+ " goalReached: " + std::to_string(goal_reached)
			+ " liedDown: " + std::to_string(sat_down)
			+ " stoodUp: " + std::to_string(stood_up);
	}
};

/**
 * @note This one is very similar to FsmLieDown
 * // TODO: try to unify these 2 FSMs
 */
class FsmSitDown: public fsmlite::fsm<FsmSitDown> {
public:
	enum State {
		MOVING_TO_GOAL = 0,
		SITTING_DOWN,
		SITTING,
		STANDING_UP,
		STANDING
	};

	FsmSitDown(State state_init = State::MOVING_TO_GOAL):
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
	void logTransition(const EventFsmSitDown& event) const {
		if (!logging_verbose_) {
			return;
		}
		HUBERO_LOG("[%s].[FsmSitDown] transition conditions\r\n%s\r\n", logger_text_.c_str(), event.toString().c_str());
	}

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
		HUBERO_LOG("[%s].[FsmSitDown] transition from MOVING TO POSITION to SITTING DOWN\r\n", logger_text_.c_str());
		logTransition(event);
	}

	void transHandlerSittingDown2Sitting(const EventFsmSitDown& event) {
		HUBERO_LOG("[%s].[FsmSitDown] transition from SITTING DOWN to SITTING\r\n", logger_text_.c_str());
		logTransition(event);
	}

	void transHandlerSitting2StandingUp(const EventFsmSitDown& event) {
		HUBERO_LOG("[%s].[FsmSitDown] transition from SITTING to STANDING UP\r\n", logger_text_.c_str());
		logTransition(event);
	}

	void transHandlerStandingUp2Standing(const EventFsmSitDown& event) {
		HUBERO_LOG("[%s].[FsmSitDown] transition from STANDING UP to STANDING\r\n", logger_text_.c_str());
		logTransition(event);
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

	bool logging_verbose_;
	std::string logger_text_;

	friend class fsmlite::fsm<FsmSitDown>;
};

} // namespace hubero
