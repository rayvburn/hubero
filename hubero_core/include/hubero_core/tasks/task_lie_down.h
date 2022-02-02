#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/tasks/fsm_lie_down.h>

namespace hubero {

/**
 * @brief Lie Down 'Object' task should not be requested for dynamic objects
 */
class TaskLieDown: public TaskEssentials<FsmLieDown, FsmLieDown::State, EventFsmLieDown> {
public:
	TaskLieDown():
		TaskEssentials::TaskEssentials(TASK_LIE_DOWN)
	{
		task_args_num_ = countArgumentsNum(&TaskLieDown::request);
		state_bb_map_ = {
			{FsmLieDown::State::MOVING_TO_GOAL, BasicBehaviourType::BB_MOVE_TO_GOAL},
			{FsmLieDown::State::LYING_DOWN, BasicBehaviourType::BB_LIE_DOWN},
			// FIXME: add basic behaviour
			{FsmLieDown::State::LYING, BasicBehaviourType::BB_STAND},
			{FsmLieDown::State::STANDING_UP, BasicBehaviourType::BB_STAND_UP_FROM_LYING},
			{FsmLieDown::State::STANDING, BasicBehaviourType::BB_STAND}
		};
	}

	virtual bool request(const Vector3& pos, const double& yaw) override {
		position_ = pos;
		yaw_ = yaw;
		return TaskEssentials::request(pos, yaw);
	}

	inline Vector3 getGoalPosition() const {
		return position_;
	}

	inline double getGoalYaw() const {
		return yaw_;
	}

protected:
	/// @brief Goal position
	Vector3 position_;
	/// @brief Yaw component of orientation required at goal position
	double yaw_;
}; // TaskLieDown

} // namespace hubero
