#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_basic.h>

namespace hubero {

/**
 * @brief Simplest task - just stand in 1 place, no goal, no FSM
 */
class TaskStand: public TaskEssentials<FsmBasic, FsmBasic::State, EventFsmBasic> {
public:
	TaskStand():
		TaskEssentials::TaskEssentials(TASK_STAND)
	{
		task_args_num_ = countArgumentsNum(&TaskStand::request);
		state_bb_map_ = {
			{FsmBasic::State::ACTIVE, BasicBehaviourType::BB_STAND},
			{FsmBasic::State::FINISHED, BasicBehaviourType::BB_STAND}
		};

		// extra operations required on state changes
		fsm_.addTransitionHandler(
			TaskStand::State::FINISHED,
			TaskStand::State::ACTIVE,
			std::bind(&TaskStand::thSetupAnimation, this, ANIMATION_STAND)
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
}; // TaskStand

} // namespace hubero
