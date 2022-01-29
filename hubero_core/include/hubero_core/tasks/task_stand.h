#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/tasks/fsm_basic.h>

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
			{FsmBasic::State::ACTIVE, BasicBehaviourType::BB_STAND}
		};
	}

	void request() {
		TaskEssentials::request();
	}
}; // TaskStand

} // namespace hubero
