#pragma once

#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_move_around.h>

namespace hubero {

class TaskMoveAround: public TaskEssentials<FsmMoveAround, FsmMoveAround::State, EventFsmMoveAround> {
public:
	TaskMoveAround(double goal_reached_distance = 1.0):
		TaskEssentials::TaskEssentials(TASK_MOVE_AROUND),
		goal_reached_distance_(goal_reached_distance)
	{
		task_args_num_ = countArgumentsNum(&TaskMoveAround::request);
		state_bb_map_ = {
			{FsmMoveAround::State::MOVING_TO_GOAL, BasicBehaviourType::BB_MOVE_TO_GOAL},
			{FsmMoveAround::State::CHOOSING_GOAL, BasicBehaviourType::BB_CHOOSE_NEW_GOAL}
		};

		// move around
		// no need to setupNavigation as MoveAround triggers goal update itself
		fsm_.addTransitionHandler(
			TaskMoveAround::State::CHOOSING_GOAL,
			TaskMoveAround::State::MOVING_TO_GOAL,
			std::bind(&TaskMoveAround::thSetupAnimation, this, ANIMATION_WALK)
		);
		fsm_.addTransitionHandler(
			TaskMoveAround::State::MOVING_TO_GOAL,
			TaskMoveAround::State::CHOOSING_GOAL,
			std::bind(&TaskMoveAround::thSetupAnimation, this, ANIMATION_STAND)
		);
	}

	virtual bool request() override {
		return TaskEssentials::request();
	}

	/// Prepare FSM event and call @ref execute
	void execute() {
		EventFsmBasic event(*this, navigation_ptr_->getFeedback());
		TaskEssentials::execute(event);
	}

	double getDistanceGoalReached() const {
		return goal_reached_distance_;
	}

protected:
	double goal_reached_distance_;

}; // TaskMoveAround

} // namespace hubero
