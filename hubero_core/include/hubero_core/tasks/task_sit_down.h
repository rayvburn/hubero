#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_sit_down.h>

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
			{FsmSitDown::State::SITTING, BasicBehaviourType::BB_SIT},
			{FsmSitDown::State::STANDING_UP, BasicBehaviourType::BB_STAND_UP_FROM_SITTING},
			{FsmSitDown::State::STANDING, BasicBehaviourType::BB_STAND}
		};

		// extra operations required on state changes
		// part 1
		fsm_.addTransitionHandler(
			FsmSitDown::State::STANDING_UP,
			FsmSitDown::State::STANDING,
			std::bind(&TaskSitDown::finish, this)
		);

		// part 2
		fsm_.addTransitionHandler(
			TaskSitDown::State::STANDING,
			TaskSitDown::State::MOVING_TO_GOAL,
			std::bind(&TaskSitDown::thSetupNavigation, this)
		);
		fsm_.addTransitionHandler(
			TaskSitDown::State::STANDING,
			TaskSitDown::State::MOVING_TO_GOAL,
			std::bind(&TaskSitDown::thSetupAnimation, this, ANIMATION_WALK)
		);
		fsm_.addTransitionHandler(
			TaskSitDown::State::MOVING_TO_GOAL,
			TaskSitDown::State::SITTING_DOWN,
			std::bind(&TaskSitDown::thSetupAnimation, this, ANIMATION_SIT_DOWN)
		);
		fsm_.addTransitionHandler(
			TaskSitDown::State::SITTING_DOWN,
			TaskSitDown::State::SITTING,
			std::bind(&TaskSitDown::thSetupAnimation, this, ANIMATION_SITTING)
		);
		fsm_.addTransitionHandler(
			TaskSitDown::State::SITTING,
			TaskSitDown::State::STANDING_UP,
			std::bind(&TaskSitDown::thSetupAnimation, this, ANIMATION_STAND_UP)
		);
		fsm_.addTransitionHandler(
			TaskSitDown::State::STANDING_UP,
			TaskSitDown::State::STANDING,
			std::bind(&TaskSitDown::thSetupAnimation, this, ANIMATION_STAND)
		);
	}

	virtual bool request(const Vector3& pos, const double& yaw) override {
		position_ = pos;
		yaw_ = yaw;
		return TaskEssentials::request(pos, yaw);
	}

	/// Prepare FSM event and call @ref execute
	void execute() {
		EventFsmSitDown event(*this, navigation_ptr_->getFeedback());
		event.setSatDown(
			animation_control_ptr_->getActiveAnimation() == AnimationType::ANIMATION_SIT_DOWN
			&& animation_control_ptr_->isFinished()
		);
		event.setStoodUp(
			animation_control_ptr_->getActiveAnimation() == AnimationType::ANIMATION_STAND_UP
			&& animation_control_ptr_->isFinished()
		);
		TaskEssentials::execute(event);
	}

	inline Vector3 getGoalPosition() const {
		return position_;
	}

	inline double getGoalYaw() const {
		return yaw_;
	}

protected:
	virtual void updateMemory() override {
		memory_ptr_->setGoal(Pose3(getGoalPosition(), Quaternion(0.0, 0.0, getGoalYaw())));
		TaskEssentials::updateMemory();
	}

	/// @brief Goal position
	Vector3 position_;
	/// @brief Yaw component of orientation required at goal position
	double yaw_;
}; // TaskSitDown

} // namespace hubero
