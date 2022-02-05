#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmFollowObject: public EventFsmBasic {
    EventFsmFollowObject() = default;

    EventFsmFollowObject(const EventFsmBasic& basic):
        EventFsmBasic(basic),
        object_nearby_(false) {}

    EventFsmFollowObject(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmFollowObject(EventFsmBasic(task, nav)) {}

    EventFsmFollowObject(const TaskPredicates& task, const NavPredicates& nav, bool object_nearby):
        EventFsmBasic(task, nav),
        object_nearby_(object_nearby) {}

    std::string toString() const {
		return EventFsmBasic::toString()
			+ " objectNearby: " + std::to_string(object_nearby_);
	}

    void setObjectNearby(bool object_nearby) {
        object_nearby_ = object_nearby;
    }

    bool isObjectNearby() const {
        return object_nearby_;
    }

protected:
	bool object_nearby_;
};

} // namespace hubero
