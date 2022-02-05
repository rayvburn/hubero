#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmTalk: public EventFsmBasic {
    EventFsmTalk() = default;

    EventFsmTalk(const EventFsmBasic& basic):
        EventFsmBasic(basic) {}

    EventFsmTalk(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmTalk(EventFsmBasic(task, nav)) {}

    std::string toString() const {
		return EventFsmBasic::toString();
	}
};

} // namespace hubero
