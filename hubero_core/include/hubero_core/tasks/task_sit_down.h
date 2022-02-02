#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/tasks/fsm_sit_down.h>

namespace hubero {

/**
 * @note This is very similar to LieDown
 */
class TaskSitDown: public TaskEssentials<FsmSitDown, FsmSitDown::State, EventFsmSitDown> {
public:
	TaskSitDown():
		TaskEssentials::TaskEssentials(TASK_SIT_DOWN)
	{
		task_args_num_ = countArgumentsNum(&TaskSitDown::request);
		state_bb_map_ = {
			{FsmSitDown::State::MOVING_TO_GOAL, BasicBehaviourType::BB_MOVE_TO_GOAL},
			{FsmSitDown::State::SITTING_DOWN, BasicBehaviourType::BB_SIT_DOWN},
			// TODO: add basic behaviour
			{FsmSitDown::State::SITTING, BasicBehaviourType::BB_SIT_DOWN},
			{FsmSitDown::State::STANDING_UP, BasicBehaviourType::BB_STAND_UP_FROM_SITTING},
			{FsmSitDown::State::STANDING, BasicBehaviourType::BB_STAND}
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
}; // TaskSitDown

} // namespace hubero
