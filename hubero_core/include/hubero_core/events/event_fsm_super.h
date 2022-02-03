#pragma once

#include <hubero_core/events/task_predicates.h>

namespace hubero {

/**
 * @brief Struct with predicates of each task
 * @details It is assumed that start of task execution erases 'requested' flag from @ref TaskPredicates
 */
struct EventFsmSuper {
	TaskPredicates stand;
	TaskPredicates move_to_goal;
	TaskPredicates move_around;
	TaskPredicates follow_object;
	TaskPredicates lie_down;
	TaskPredicates sit_down;
	TaskPredicates run;
	TaskPredicates talk;
	TaskPredicates teleop;

	std::string toString() const {
		return "\tstand:        " + stand.toString() + "\r\n"
			+ "\tmoveToGoal:   " + move_to_goal.toString() + "\r\n"
			+ "\tmoveAround:   " + move_around.toString() + "\r\n"
			+ "\tfollowObject: " + follow_object.toString() + "\r\n"
			+ "\tlieDown:      " + lie_down.toString() + "\r\n"
			+ "\tsitDown:      " + sit_down.toString() + "\r\n"
			+ "\trun:          " + run.toString() + "\r\n"
			+ "\ttalk:         " + talk.toString() + "\r\n"
			+ "\tteleop:       " + teleop.toString()
		;
	}
};

} // namespace hubero
