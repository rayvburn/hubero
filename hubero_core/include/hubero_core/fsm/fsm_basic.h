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
        /// some navigation-related predicate triggered task finishing
        FINISHED
	};

	FsmBasic(State state_init = State::ACTIVE): fsm(state_init), FsmEssentials("FsmBasic") {}

protected:
	/**
	 * @defgroup guards FSM transition guards
	 * @{
	 */
	bool guardActive2Finished(const EventFsmBasic& event) const {
		return event.nav_succeeded || event.nav_cancelled || event.nav_rejected;
	}

	bool guardFinished2Active(const EventFsmBasic& event) const {
		return event.active;// && (event.nav_succeeded || event.nav_cancelled || event.nav_rejected);
	}

	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerActive2Finished(const EventFsmBasic& event) {
		logTransition("ACTIVE", "FINISHED", event);
	}

	void transHandlerFinished2Active(const EventFsmBasic& event) {
		logTransition("FINISHED", "ACTIVE", event);
	}

	/** @} */ // end of transition handlers group

	using transition_table = table<
        mem_fn_row<State::ACTIVE, EventFsmBasic, State::FINISHED, &FsmBasic::transHandlerActive2Finished, &FsmBasic::guardActive2Finished>,
		mem_fn_row<State::FINISHED, EventFsmBasic, State::ACTIVE, &FsmBasic::transHandlerFinished2Active, &FsmBasic::guardFinished2Active>
    >;

	friend class fsmlite::fsm<FsmBasic>;
};

} // namespace hubero
