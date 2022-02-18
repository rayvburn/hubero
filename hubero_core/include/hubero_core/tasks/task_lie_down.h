#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_lie_down.h>

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
			{FsmLieDown::State::LYING, BasicBehaviourType::BB_LIE},
			{FsmLieDown::State::STANDING_UP, BasicBehaviourType::BB_STAND_UP_FROM_LYING},
			{FsmLieDown::State::STANDING, BasicBehaviourType::BB_STAND}
		};

		fsm_.addTransitionHandler(
			FsmLieDown::State::STANDING_UP,
			FsmLieDown::State::STANDING,
			std::bind(&TaskLieDown::finish, this)
		);

		// part2
		fsm_.addTransitionHandler(
			TaskLieDown::State::STANDING,
			TaskLieDown::State::MOVING_TO_GOAL,
			std::bind(&TaskLieDown::thSetupNavigation, this)
		);
		fsm_.addTransitionHandler(
			TaskLieDown::State::STANDING,
			TaskLieDown::State::MOVING_TO_GOAL,
			std::bind(&TaskLieDown::thSetupAnimation, this, ANIMATION_WALK)
		);
		fsm_.addTransitionHandler(
			TaskLieDown::State::MOVING_TO_GOAL,
			TaskLieDown::State::LYING_DOWN,
			std::bind(&TaskLieDown::thSetupAnimation, this, ANIMATION_LIE_DOWN)
		);
		fsm_.addTransitionHandler(
			TaskLieDown::State::LYING_DOWN,
			TaskLieDown::State::LYING,
			std::bind(&TaskLieDown::thSetupAnimation, this, ANIMATION_LYING)
		);
		fsm_.addTransitionHandler(
			TaskLieDown::State::LYING,
			TaskLieDown::State::STANDING_UP,
			std::bind(&TaskLieDown::thSetupAnimation, this, ANIMATION_STAND_UP)
		);
		fsm_.addTransitionHandler(
			TaskLieDown::State::STANDING_UP,
			TaskLieDown::State::STANDING,
			std::bind(&TaskLieDown::thSetupAnimation, this, ANIMATION_STAND)
		);
	}

	virtual bool request(const Vector3& pos, const double& yaw) override {
		position_ = pos;
		yaw_ = yaw;
		return TaskEssentials::request(pos, yaw);
	}

	/// Prepare FSM event and call @ref execute
	void execute() {
		EventFsmLieDown event(*this, navigation_ptr_->getFeedback());
		event.setLiedDown(
			animation_control_ptr_->getActiveAnimation() == AnimationType::ANIMATION_LIE_DOWN
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
}; // TaskLieDown

} // namespace hubero
