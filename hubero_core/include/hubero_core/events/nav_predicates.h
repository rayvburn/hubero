#pragma once

#include <hubero_common/defines.h>
#include <string>

namespace hubero {

struct NavPredicates {
    /// @brief Default constructor
    NavPredicates():
        nav_rejected_(false),
        nav_cancelled_(false),
        nav_active_(false),
        nav_succeeded_(false) {}

    /// @brief Constructor that explicitly takes all flags
    NavPredicates(bool nav_rejected, bool nav_cancelled, bool nav_active, bool nav_succeeded):
        nav_rejected_(nav_rejected),
        nav_cancelled_(nav_cancelled),
        nav_active_(nav_active),
        nav_succeeded_(nav_succeeded) {}

    /// @brief Constructor that updates internal flags based on navigation task feedback
    NavPredicates(const TaskFeedbackType& feedback):
        nav_rejected_(feedback == TaskFeedbackType::TASK_FEEDBACK_REJECTED),
        nav_cancelled_(feedback == TaskFeedbackType::TASK_FEEDBACK_PREEMPTING
            || feedback == TaskFeedbackType::TASK_FEEDBACK_PREEMPTED
            || feedback == TaskFeedbackType::TASK_FEEDBACK_ABORTED
            || feedback == TaskFeedbackType::TASK_FEEDBACK_LOST
            || feedback == TaskFeedbackType::TASK_FEEDBACK_RECALLED
            || feedback == TaskFeedbackType::TASK_FEEDBACK_RECALLING),
        nav_active_(feedback == TaskFeedbackType::TASK_FEEDBACK_ACTIVE),
        nav_succeeded_(feedback == TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED),
        nav_ended_(feedback == TaskFeedbackType::TASK_FEEDBACK_TERMINATED) {}

    bool isNavigationGoalRejected() const {
        return nav_rejected_ && !nav_active_ && !nav_cancelled_ && !nav_succeeded_ && !nav_ended_;
    }

    bool isNavigationActive() const {
		return !nav_rejected_ && nav_active_ && !nav_cancelled_ && !nav_succeeded_ && !nav_ended_;
	}

    bool isNavigationGoalCancelled() const {
        return !nav_rejected_ && !nav_active_ && nav_cancelled_ && !nav_succeeded_ && !nav_ended_;
    }

    bool isNavigationSucceeded() const {
        return !nav_rejected_ && !nav_active_ && !nav_cancelled_ && nav_succeeded_ && !nav_ended_;
    }

    bool isNavigationEnded() const {
        return !nav_rejected_ && !nav_active_ && !nav_cancelled_ && !nav_succeeded_ && nav_ended_;
    }

    std::string toString() const {
		return "reject " + std::to_string(nav_rejected_)
			+ " cancel " + std::to_string(nav_cancelled_)
            + " active " + std::to_string(nav_active_)
			+ " succeed " + std::to_string(nav_succeeded_)
            + " ended " + std::to_string(nav_ended_);
	}

protected:
	/// navigation task aborted before execution
    bool nav_rejected_;
    /// navigation task aborted during execution
	bool nav_cancelled_;
    /// navigation task is currently being executed
    bool nav_active_;
	/// navigation task finished successfully
    bool nav_succeeded_;
    /// navigation status resetted by external call
    bool nav_ended_;
};

} // namespace hubero
