#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmSitDown: public EventFsmBasic {
	EventFsmSitDown() = default;

    EventFsmSitDown(const EventFsmBasic& basic):
        EventFsmBasic(basic),
        sat_down_(false),
        stood_up_(false) {}

	EventFsmSitDown(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmSitDown(EventFsmBasic(task, nav)) {}

    EventFsmSitDown(const TaskPredicates& task, const NavPredicates& nav, bool sat_down, bool stood_up):
        EventFsmBasic(task, nav),
		sat_down_(sat_down),
		stood_up_(stood_up) {}

	std::string toString() const {
		return EventFsmBasic::toString()
			+ " liedDown: " + std::to_string(sat_down_)
			+ " stoodUp: " + std::to_string(stood_up_);
	}

	void setSatDown(bool sat_down) {
		sat_down_ = sat_down;
	}

	void setStoodUp(bool stood_up) {
		stood_up_ = stood_up;
	}

	bool hasSatDown() const {
		return sat_down_;
	}

	bool hasStoodUp() const {
		return stood_up_;
	}

protected:
	bool sat_down_;
	bool stood_up_;
};

} // namespace hubero
