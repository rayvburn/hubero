#pragma once

#include <string>

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
};

} // namespace hubero
