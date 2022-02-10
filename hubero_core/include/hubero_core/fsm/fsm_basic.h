#pragma once

#include <hubero_core/fsm/fsm_essentials.h>
#include <hubero_core/events/event_fsm_basic.h>

// fsmlite
#include <fsm.h>

namespace hubero {

class FsmBasic: public fsmlite::fsm<FsmBasic>, public FsmEssentials {
public:
	enum State {
        /// normal operation - motion execution based on plan
		ACTIVE = 0,
        /// some navigation-related predicate notify task finished; kind of an interface between low level FSM and HFSM
        FINISHED
	};

	/**
	 * @brief Basic FSM (active/finished manner) constructor
	 *
	 * @note default initial state set to FINISHED as it will produce typical operation from the start (1st execution
	 * not needed to be handled separately)
	 */
	FsmBasic(State state_init = State::FINISHED): fsm(state_init), FsmEssentials("FsmBasic") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardActive2Finished(const EventFsmBasic& event) const {
		// NOTE: isNavigationEnded deleted since it will produce periodic state changes in tasks that not use
		// navigation goal as a finish rule
		// NOTE2: isNavigationGoalCancelled deleted since it's called each time goal gets updated; nevertheless, task
		// will be terminated by higher level FSM
		return event.isNavigationSucceeded()
			|| event.isNavigationGoalRejected()
			|| event.isAborted();
	}

	bool guardFinished2Active(const EventFsmBasic& event) const {
		return !event.isNavigationActive() && event.isActive();
	}

	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerActive2Finished(const EventFsmBasic& event) {
		logTransition("ACTIVE", "FINISHED", event);
		transitionHandler(State::ACTIVE, State::FINISHED);
	}

	void transHandlerFinished2Active(const EventFsmBasic& event) {
		logTransition("FINISHED", "ACTIVE", event);
		transitionHandler(State::FINISHED, State::ACTIVE);
	}

	/** @} */ // end of transition handlers group

	using transition_table = table<
        mem_fn_row<State::ACTIVE, EventFsmBasic, State::FINISHED, &FsmBasic::transHandlerActive2Finished, &FsmBasic::guardActive2Finished>,
		mem_fn_row<State::FINISHED, EventFsmBasic, State::ACTIVE, &FsmBasic::transHandlerFinished2Active, &FsmBasic::guardFinished2Active>
    >;

	friend class fsmlite::fsm<FsmBasic>;
};

} // namespace hubero
