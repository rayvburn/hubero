#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmLieDown: public EventFsmBasic {
	bool goal_reached;
	bool lied_down;
	bool stood_up;

	std::string toString() const {
		return EventFsmBasic::toString()
			+ " goalReached: " + std::to_string(goal_reached)
			+ " liedDown: " + std::to_string(lied_down)
			+ " stoodUp: " + std::to_string(stood_up);
	}

    EventFsmLieDown() = default;

    EventFsmLieDown(const EventFsmBasic& basic):
        EventFsmBasic(basic),
        goal_reached(false),
        lied_down(false),
        stood_up(false) {}

    EventFsmLieDown(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmLieDown(EventFsmBasic(task, nav)) {}
};

} // namespace hubero
