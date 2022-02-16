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
		return std::string("\r\n")
			+ "\tstand:        " + stand.toString() + "\r\n"
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

	EventFsmSuper() = default;

	EventFsmSuper(
		const std::shared_ptr<const TaskBase> task_follow_object_ptr,
		const std::shared_ptr<const TaskBase> task_lie_down_ptr,
		const std::shared_ptr<const TaskBase> task_move_around_ptr,
		const std::shared_ptr<const TaskBase> task_move_to_goal_ptr,
		const std::shared_ptr<const TaskBase> task_run_ptr,
		const std::shared_ptr<const TaskBase> task_sit_down_ptr,
		const std::shared_ptr<const TaskBase> task_stand_ptr,
		const std::shared_ptr<const TaskBase> task_talk_ptr,
		const std::shared_ptr<const TaskBase> task_teleop_ptr
	):
		follow_object(TaskPredicates(task_follow_object_ptr)),
		lie_down(TaskPredicates(task_lie_down_ptr)),
		move_around(TaskPredicates(task_move_around_ptr)),
		move_to_goal(TaskPredicates(task_move_to_goal_ptr)),
		run(TaskPredicates(task_run_ptr)),
		sit_down(TaskPredicates(task_sit_down_ptr)),
		stand(TaskPredicates(task_stand_ptr)),
		talk(TaskPredicates(task_talk_ptr)),
		teleop(TaskPredicates(task_teleop_ptr)) {}
};

} // namespace hubero
