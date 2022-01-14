#include <hubero_ros/task_request_ros_api.h>
#include <hubero_ros/task_request_ros.h>

#include <iostream>

namespace hubero {

TaskRequestRosApi::TaskRequestRosApi(const std::string& actor_name):
	node_ptr_(std::make_shared<Node>("task_request_ros_api_node")
) {
	// base name, then specified by specific task names
	std::string actor_task_ns =
		"/"
		+ node_ptr_->getNamespaceName()
		+ "/" + actor_name
		+ "/" + node_ptr_->getTaskNamespaceName()
		+ "/";

	ac_follow_object_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::FollowObjectAction,
		hubero_ros_msgs::FollowObjectActionFeedbackConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_FOLLOW_OBJECT)
	);

	ac_lie_down_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::LieDownAction,
		hubero_ros_msgs::LieDownActionFeedbackConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_LIE_DOWN)
	);

	ac_lie_down_object_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::LieDownObjectAction,
		hubero_ros_msgs::LieDownObjectActionFeedbackConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_LIE_DOWN) + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX
	);

	ac_move_to_goal_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::MoveToGoalAction,
		hubero_ros_msgs::MoveToGoalActionFeedbackConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_MOVE_TO_GOAL)
	);

	ac_move_to_object_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::MoveToObjectAction,
		hubero_ros_msgs::MoveToObjectActionFeedbackConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_MOVE_TO_GOAL) + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX
	);
}

bool TaskRequestRosApi::followObject(const std::string& object_name) {
	hubero_ros_msgs::FollowObjectGoal action_goal;
	action_goal.object_name = object_name;
	ac_follow_object_ptr_->sendGoal(action_goal);
}

bool TaskRequestRosApi::lieDown() {
	hubero_ros_msgs::LieDownGoal action_goal;
	// TODO: specify goal
	ac_lie_down_ptr_->sendGoal(action_goal);
}

bool TaskRequestRosApi::lieDownObject(const std::string& object_name) {
	hubero_ros_msgs::LieDownObjectGoal action_goal;
	// TODO: specify goal
	ac_lie_down_object_ptr_->sendGoal(action_goal);
}

bool TaskRequestRosApi::moveToGoal() {
	hubero_ros_msgs::MoveToGoalGoal action_goal;
	// TODO: specify goal
	ac_move_to_goal_ptr_->sendGoal(action_goal);
}

bool TaskRequestRosApi::moveToObject(const std::string& object_name) {
	hubero_ros_msgs::MoveToObjectGoal action_goal;
	action_goal.object_name = object_name;
	ac_move_to_object_ptr_->sendGoal(action_goal);
}

} // namespace hubero
