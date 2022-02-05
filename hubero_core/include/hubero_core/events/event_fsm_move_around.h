#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmMoveAround: public EventFsmBasic {
    EventFsmMoveAround() = default;

    EventFsmMoveAround(const EventFsmBasic& basic):
        EventFsmBasic(basic) {}

    EventFsmMoveAround(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmMoveAround(EventFsmBasic(task, nav)) {}

    std::string toString() const {
		return EventFsmBasic::toString();
	}
};

} // namespace hubero
