#pragma once

namespace hubero {

enum AnimationType {
	ANIMATION_STAND = 0,
	ANIMATION_WALK,
	ANIMATION_LIE_DOWN,
	ANIMATION_SIT_DOWN,
	ANIMATION_SITTING,
	ANIMATION_STAND_UP,
	ANIMATION_RUN,
	ANIMATION_TALK
};

enum TaskType {
	TASK_STAND = 0,
	TASK_MOVE_TO_GOAL,
	TASK_MOVE_AROUND,
	TASK_LIE_DOWN,
	TASK_SIT_DOWN,
	TASK_FOLLOW_OBJECT,
	TASK_TELEOP,
	TASK_RUN,
	TASK_TALK
};

/// Description based on ROS GoalStatus message definition
enum TaskFeedbackType {
	/// The goal has yet to be processed
	TASK_FEEDBACK_PENDING = 0,
	/// The goal is currently being processed by the action server
	TASK_FEEDBACK_ACTIVE,
	/// The goal received a cancel request after it started executing and has since completed its execution
	TASK_FEEDBACK_PREEMPTED,
	/// The goal was achieved successfully
	TASK_FEEDBACK_SUCCEEDED,
	/// The goal was aborted during execution by the action server due to some failure
	TASK_FEEDBACK_ABORTED,
	/// The goal was rejected by the action server without being processed, because the goal was unattainable
	/// or invalid
	TASK_FEEDBACK_REJECTED,
	/// The goal received a cancel request after it started executing and has not yet completed execution
	TASK_FEEDBACK_PREEMPTING,
	/// The goal received a cancel request before it started executing, but the action server has not yet confirmed
	/// that the goal is canceled
	TASK_FEEDBACK_RECALLING,
	/// The goal received a cancel request before it started executing and was successfully cancelled
	TASK_FEEDBACK_RECALLED,
	/// An action client can determine that a goal is LOST
	TASK_FEEDBACK_LOST,
	/// Undefined state
	TASK_FEEDBACK_UNDEFINED = 127
};

enum BasicBehaviourType {
	BB_UNDEFINED = 0,
	BB_STAND,
	BB_ALIGN_TO_TARGET,
	BB_MOVE_TO_GOAL,
	BB_MOVE_AROUND,
	BB_LIE_DOWN,
	BB_FOLLOW_OBJECT,
	BB_TELEOP,
	BB_TALK,
	BB_SIT_DOWN
};

} // namespace hubero