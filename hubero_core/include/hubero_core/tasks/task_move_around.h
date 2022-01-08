#pragma once

#include <hubero_interfaces/utils/task_base.h>

namespace hubero {

class TaskMoveAround: public TaskBase {
public:
	enum State {
		STATE_MOVE_TO_GOAL = 0,
		STATE_CHOOSE_NEW_GOAL
	};

	TaskMoveAround();
}; // TaskMoveAround

} // namespace hubero
