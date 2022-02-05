#pragma once

#include <hubero_core/events/event_fsm_basic.h>

namespace hubero {

struct EventFsmLieDown: public EventFsmBasic {
	EventFsmLieDown() = default;

    EventFsmLieDown(const EventFsmBasic& basic):
        EventFsmBasic(basic),
        lied_down_(false),
        stood_up_(false) {}

    EventFsmLieDown(const TaskPredicates& task, const NavPredicates& nav):
        EventFsmLieDown(EventFsmBasic(task, nav)) {}

	EventFsmLieDown(const TaskPredicates& task, const NavPredicates& nav, bool lied_down, bool stood_up):
        EventFsmBasic(task, nav),
		lied_down_(lied_down),
		stood_up_(stood_up) {}

	std::string toString() const {
		return EventFsmBasic::toString()
			+ " liedDown: " + std::to_string(lied_down_)
			+ " stoodUp: " + std::to_string(stood_up_);
	}

	void setLiedDown(bool lied_down) {
		lied_down_ = lied_down;
	}

	void setStoodUp(bool stood_up) {
		stood_up_ = stood_up;
	}

	bool hasLiedDown() const {
		return lied_down_;
	}

	bool hasStoodUp() const {
		return stood_up_;
	}

protected:
	bool lied_down_;
	bool stood_up_;
};

} // namespace hubero
