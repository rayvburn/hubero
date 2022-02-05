#pragma once

#include <hubero_interfaces/utils/task_base.h>
#include <memory>
#include <string>

namespace hubero {

struct TaskPredicates {
	/// @brief Default constructor
	TaskPredicates():
		requested_(false),
		active_(false),
		aborted_(false),
		finished_(false) {}

	/// @brief Constructor that explicitly takes all flags
	TaskPredicates(bool requested, bool active, bool aborted, bool finished):
		requested_(requested),
		active_(active),
		aborted_(aborted),
		finished_(finished) {}

	/// @brief Constructor that updates internal flags based on taken task object reference
	TaskPredicates(const TaskBase& task):
		requested_(task.isRequested()),
		active_(task.isActive()),
		aborted_(task.isAborted()),
		finished_(task.isFinished()) {}

	/// @brief Constructor that updates internal flags taking pointer to class derived from TaskBase
	TaskPredicates(const std::shared_ptr<const TaskBase> task_ptr):
		requested_(task_ptr->isRequested()),
		active_(task_ptr->isActive()),
		aborted_(task_ptr->isAborted()),
		finished_(task_ptr->isFinished()) {}

	bool isPending() const {
		return requested_ && !active_ && !aborted_ && !finished_;
	}

	bool isActive() const {
		return !requested_ && active_ && !aborted_ && !finished_;
	}

	/// @brief Task ended, either with a success or due to abort
	bool isEnded() const {
		return (finished_ || aborted_) && !requested_ && !active_;
	}

	/// @brief Task ended with a success
	bool isSucceeded() const {
		return finished_ && !aborted_ && !requested_ && !active_;
	}

	bool isAborted() const {
		return aborted_;
	}

	bool isTerminated() const {
		return !active_ && !aborted_ && !finished_ && !requested_;
	}

	std::string toString() const {
		return "req " + std::to_string(requested_)
			+ " active " + std::to_string(active_)
			+ " aborted " + std::to_string(aborted_)
			+ " finished " + std::to_string(finished_);
	}

protected:
	bool requested_;
	bool active_;
	bool aborted_;
	bool finished_;
};

} // namespace hubero
