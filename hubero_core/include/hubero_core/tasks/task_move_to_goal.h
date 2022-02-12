#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_basic.h>

namespace hubero {

/**
 * @brief Helps executing 'move to goal' task
 * @note Tasks consisting of 1 basic behaviour do not need FSM
 */
class TaskMoveToGoal: public TaskEssentials<FsmBasic, FsmBasic::State, EventFsmBasic> {
public:
	TaskMoveToGoal(): TaskEssentials::TaskEssentials(TASK_MOVE_TO_GOAL) {
		task_args_num_ = countArgumentsNum(&TaskMoveToGoal::request);
		state_bb_map_ = {
			{FsmBasic::State::ACTIVE, BasicBehaviourType::BB_MOVE_TO_GOAL},
			{FsmBasic::State::FINISHED, BasicBehaviourType::BB_STAND}
		};

		// this task should not be looped infinitely - once goal is achieved, task should be finished
		fsm_.addTransitionHandler(
			FsmBasic::State::ACTIVE,
			FsmBasic::State::FINISHED,
			std::bind(&TaskMoveToGoal::finish, this)
		);
	}

	void updateMemory(InternalMemory& memory, const std::shared_ptr<const WorldGeometryBase> world_geometry_ptr) {
		memory.setGoal(getGoal());
		TaskEssentials::updateMemory(memory, world_geometry_ptr);
	}

	virtual bool request(const Pose3& goal) override {
		goal_ = goal;
		return TaskEssentials::request(goal);
	}

	inline Pose3 getGoal() const {
		return goal_;
	}

protected:
	Pose3 goal_;
}; // class TaskStand

} // namespace hubero
