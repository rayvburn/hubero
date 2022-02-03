#pragma once

#include <hubero_common/defines.h>
#include <string>

namespace hubero {

struct NavPredicates {
	/// navigation task aborted before execution
    bool nav_rejected;
    /// navigation task aborted during execution
	bool nav_cancelled;
    /// navigation task is currently being executed
    bool nav_active;
	/// navigation task finished successfully
    bool nav_succeeded;

    bool isNavigationActive() {
		return nav_active && !nav_succeeded && !nav_cancelled && !nav_rejected;
	}

    std::string toString() const {
		return "nav: reject " + std::to_string(nav_rejected)
			+ " cancel " + std::to_string(nav_cancelled)
            + " active " + std::to_string(nav_active)
			+ " succeed " + std::to_string(nav_succeeded);
	}

    NavPredicates():
        nav_rejected(false),
        nav_cancelled(false),
        nav_active(false),
        nav_succeeded(false) {}

    NavPredicates(TaskFeedbackType feedback):
        nav_rejected(feedback == TaskFeedbackType::TASK_FEEDBACK_REJECTED),
        nav_cancelled(feedback == TaskFeedbackType::TASK_FEEDBACK_PREEMPTING
            || feedback == TaskFeedbackType::TASK_FEEDBACK_PREEMPTED
            || feedback == TaskFeedbackType::TASK_FEEDBACK_ABORTED
            || feedback == TaskFeedbackType::TASK_FEEDBACK_LOST),
        nav_active(feedback == TaskFeedbackType::TASK_FEEDBACK_ACTIVE),
        nav_succeeded(feedback == TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED) {}
};

} // namespace hubero
