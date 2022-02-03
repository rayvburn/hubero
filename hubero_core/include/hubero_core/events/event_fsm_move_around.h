#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmMoveAround: public EventFsmBasic {
	bool goal_reached;
	bool goal_selected;

	std::string toString() const {
		return EventFsmBasic::toString()
			+ " goalReached: " + std::to_string(goal_reached)
			+ " goalSelected: " + std::to_string(goal_selected);
	}

    EventFsmMoveAround() = default;

    EventFsmMoveAround(const EventFsmBasic& basic):
        EventFsmBasic(basic),
        goal_reached(false),
        goal_selected(false) {}

    EventFsmMoveAround(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmMoveAround(EventFsmBasic(task, nav)) {}
};

} // namespace hubero
