#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_basic.h>

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
			{FsmBasic::State::ACTIVE, BasicBehaviourType::BB_RUN},
			{FsmBasic::State::FINISHED, BasicBehaviourType::BB_STAND}
		};

		// this task should not be looped infinitely - once goal is achieved, task should be finished
		fsm_.addTransitionHandler(
			FsmBasic::State::ACTIVE,
			FsmBasic::State::FINISHED,
			std::bind(&TaskRun::finish, this)
		);

		// run to goal
		fsm_.addTransitionHandler(
			TaskRun::State::FINISHED,
			TaskRun::State::ACTIVE,
			std::bind(&TaskRun::thSetupNavigation, this)
		);
		fsm_.addTransitionHandler(
			TaskRun::State::FINISHED,
			TaskRun::State::ACTIVE,
			std::bind(&TaskRun::thSetupAnimation, this, ANIMATION_RUN)
		);
		fsm_.addTransitionHandler(
			TaskRun::State::ACTIVE,
			TaskRun::State::FINISHED,
			std::bind(&TaskRun::thSetupAnimation, this, ANIMATION_STAND)
		);
	}

	virtual bool request(const Pose3& goal) override {
		goal_ = goal;
		return TaskEssentials::request(goal);
	}

	/// Prepare FSM event and call @ref execute
	void execute() {
		EventFsmBasic event(*this, navigation_ptr_->getFeedback());
		TaskEssentials::execute(event);
	}

	inline Pose3 getGoal() const {
		return goal_;
	}

	double getDistanceGoalReached() const {
		return goal_reached_distance_;
	}

protected:
	virtual void updateMemory() override {
		memory_ptr_->setGoal(getGoal());
		TaskEssentials::updateMemory();
	}

	Pose3 goal_;

	double goal_reached_distance_;
}; // TaskRun

} // namespace hubero
