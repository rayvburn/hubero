#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_talk.h>

namespace hubero {

class TaskTalk: public TaskEssentials<FsmTalk, FsmTalk::State, EventFsmTalk> {
public:
	TaskTalk(double goal_reached_distance = 1.0):
		TaskEssentials::TaskEssentials(TASK_TALK),
		goal_reached_distance_(goal_reached_distance)
	{
		task_args_num_ = countArgumentsNum(&TaskTalk::request);
		state_bb_map_ = {
			{FsmTalk::State::MOVING_TO_GOAL, BasicBehaviourType::BB_MOVE_TO_GOAL},
			{FsmTalk::State::TALKING, BasicBehaviourType::BB_TALK}
		};
	}

	virtual bool request(const Pose3& goal) override {
		goal_ = goal;
		return TaskEssentials::request(goal);
	}

	/// Prepare FSM event and call @ref execute
	void execute() {
		EventFsmTalk event(*this, navigation_ptr_->getFeedback());
		TaskEssentials::execute(event);
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
}; // TaskTalk

} // namespace hubero
