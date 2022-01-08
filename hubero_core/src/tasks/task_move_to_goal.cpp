#include <hubero_core/tasks/task_move_to_goal.h>

namespace hubero {

TaskMoveToGoal::TaskMoveToGoal(): TaskBase::TaskBase(TASK_MOVE_TO_GOAL) {}

void TaskMoveToGoal::request(const Pose3& goal) {
	goal_ = goal;
	TaskBase::request<const Pose3&>(goal);
}

} // namespace hubero
