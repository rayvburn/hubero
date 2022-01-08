#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_interfaces/utils/task_base.h>

namespace hubero {

/**
 * @brief Helps executing 'move to goal' task
 * @note Tasks consisting of 1 basic behaviour do not need FSM
 */
class TaskMoveToGoal: public TaskBase {
public:
	TaskMoveToGoal();

	virtual void request(const Pose3& goal);

	inline Pose3 getGoal() const {
		return goal_;
	}

protected:
	Pose3 goal_;
}; // class TaskStand

} // namespace hubero
