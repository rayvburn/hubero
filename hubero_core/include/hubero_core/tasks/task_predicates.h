#pragma once

#include <string>
#include <hubero_interfaces/utils/task_base.h>

namespace hubero {

struct TaskPredicates {
	bool requested;
	bool active;
	bool aborted;
	bool finished;

	std::string toString() const {
		return "req " + std::to_string(requested)
			+ " active " + std::to_string(active)
			+ " aborted " + std::to_string(aborted)
			+ " finished " + std::to_string(finished);
	}

	/// @brief Default constructor
	TaskPredicates():
		requested(false),
		active(false),
		aborted(false),
		finished(false) {}

	/// @brief Constructor that takes updates internal flags based on task object
	TaskPredicates(const TaskBase& task):
		requested(task.isRequested()),
		active(task.isActive()),
		aborted(task.isAborted()),
		finished(task.isFinished()) {}
};

} // namespace hubero
