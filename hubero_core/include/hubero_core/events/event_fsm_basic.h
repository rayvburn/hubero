#pragma once

#include <hubero_core/events/task_predicates.h>
#include <hubero_core/events/nav_predicates.h>

namespace hubero {

struct EventFsmBasic: public TaskPredicates, public NavPredicates {
	std::string toString() const {
		return "task {" + TaskPredicates::toString() + "} nav {" + NavPredicates::toString() + "}";
	}

	EventFsmBasic(): TaskPredicates(), NavPredicates() {}

	EventFsmBasic(const TaskPredicates& task, const NavPredicates& nav):
		TaskPredicates(task),
		NavPredicates(nav) {}
};

} // namespace hubero
