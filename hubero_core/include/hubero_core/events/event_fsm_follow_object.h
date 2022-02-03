#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmFollowObject: public EventFsmBasic {
	bool object_nearby;

	std::string toString() const {
		return EventFsmBasic::toString()
			+ " objectNearby: " + std::to_string(object_nearby);
	}

    EventFsmFollowObject() = default;

    EventFsmFollowObject(const EventFsmBasic& basic):
        EventFsmBasic(basic),
        object_nearby(false) {}

    EventFsmFollowObject(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmFollowObject(EventFsmBasic(task, nav)) {}
};

} // namespace hubero
