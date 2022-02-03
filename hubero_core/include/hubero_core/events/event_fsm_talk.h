#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmTalk: public EventFsmBasic {
	bool goal_reached;

	std::string toString() const {
		return EventFsmBasic::toString()
			+ " goalReached: " + std::to_string(goal_reached);
	}

    EventFsmTalk() = default;

    EventFsmTalk(const EventFsmBasic& basic):
        EventFsmBasic(basic),
        goal_reached(false) {}

    EventFsmTalk(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmTalk(EventFsmBasic(task, nav)) {}
};

} // namespace hubero
