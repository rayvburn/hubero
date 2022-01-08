#pragma once

#include <hubero_interfaces/utils/task_base.h>

namespace hubero {

/**
 * @brief Simplest task - just stand in 1 place, no goal, no FSM
 */
class TaskStand: public TaskBase {
public:
	TaskStand();
}; // TaskStand

} // namespace hubero
