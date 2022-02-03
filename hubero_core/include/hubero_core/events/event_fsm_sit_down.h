#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmSitDown: public EventFsmBasic {
	bool goal_reached;
	bool sat_down;
	bool stood_up;

	std::string toString() const {
		return EventFsmBasic::toString()
			+ " goalReached: " + std::to_string(goal_reached)
			+ " liedDown: " + std::to_string(sat_down)
			+ " stoodUp: " + std::to_string(stood_up);
	}

    EventFsmSitDown() = default;

    EventFsmSitDown(const EventFsmBasic& basic):
        EventFsmBasic(basic),
        goal_reached(false),
        sat_down(false),
        stood_up(false) {}

    EventFsmSitDown(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmSitDown(EventFsmBasic(task, nav)) {}
};

} // namespace hubero
