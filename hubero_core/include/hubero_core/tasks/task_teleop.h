#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_basic.h>

namespace hubero {

class TaskTeleop: public TaskEssentials<FsmBasic, FsmBasic::State, EventFsmBasic> {
public:
	TaskTeleop():
		TaskEssentials::TaskEssentials(TASK_TELEOP)
	{
		task_args_num_ = countArgumentsNum(&TaskTeleop::request);
		state_bb_map_ = {
			{FsmBasic::State::ACTIVE, BasicBehaviourType::BB_TELEOP},
			{FsmBasic::State::FINISHED, BasicBehaviourType::BB_STAND}
		};
	}

	virtual bool request() override {
		return TaskEssentials::request();
	}

	void setCommand(const Vector3& cmd) {
		cmd_ = cmd;
	}

	Vector3 getCommand() const {
		return cmd_;
	}

protected:
	Vector3 cmd_;
}; // TaskTeleop

} // namespace hubero
