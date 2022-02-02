#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/tasks/fsm_basic.h>

namespace hubero {

/**
 * @note In fact this is a MoveToGoal task but with different animation and different movement speed
 * // TODO: inherit from TaskMoveToGoal
 */
class TaskRun: public TaskEssentials<FsmBasic, FsmBasic::State, EventFsmBasic> {
public:
	TaskRun(double goal_reached_distance = 1.0):
		TaskEssentials::TaskEssentials(TASK_RUN),
		goal_reached_distance_(goal_reached_distance)
	{
		task_args_num_ = countArgumentsNum(&TaskRun::request);
		state_bb_map_ = {
			{FsmBasic::State::ACTIVE, BasicBehaviourType::BB_RUN}
		};
	}

	virtual bool request(const Pose3& goal) override {
		goal_ = goal;
		return TaskEssentials::request(goal);
	}

	inline Pose3 getGoal() const {
		return goal_;
	}

	double getDistanceGoalReached() const {
		return goal_reached_distance_;
	}

protected:
	Pose3 goal_;

	double goal_reached_distance_;
}; // TaskRun

} // namespace hubero
