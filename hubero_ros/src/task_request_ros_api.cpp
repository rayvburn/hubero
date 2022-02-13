#include <hubero_ros/task_request_ros_api.h>
#include <hubero_ros/task_request_ros.h>
#include <hubero_ros/utils/converter.h>

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
		hubero_ros_msgs::FollowObjectActionFeedbackConstPtr,
		hubero_ros_msgs::FollowObjectActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_FOLLOW_OBJECT)
	);

	ac_lie_down_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::LieDownAction,
		hubero_ros_msgs::LieDownActionFeedbackConstPtr,
		hubero_ros_msgs::LieDownActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_LIE_DOWN)
	);

	ac_lie_down_object_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::LieDownObjectAction,
		hubero_ros_msgs::LieDownObjectActionFeedbackConstPtr,
		hubero_ros_msgs::LieDownObjectActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_LIE_DOWN) + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX
	);

	ac_move_around_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::MoveAroundAction,
		hubero_ros_msgs::MoveAroundActionFeedbackConstPtr,
		hubero_ros_msgs::MoveAroundActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_MOVE_AROUND)
	);

	ac_move_to_goal_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::MoveToGoalAction,
		hubero_ros_msgs::MoveToGoalActionFeedbackConstPtr,
		hubero_ros_msgs::MoveToGoalActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_MOVE_TO_GOAL)
	);

	ac_move_to_object_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::MoveToObjectAction,
		hubero_ros_msgs::MoveToObjectActionFeedbackConstPtr,
		hubero_ros_msgs::MoveToObjectActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_MOVE_TO_GOAL) + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX
	);

	ac_run_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::RunAction,
		hubero_ros_msgs::RunActionFeedbackConstPtr,
		hubero_ros_msgs::RunActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_RUN)
	);

	ac_sit_down_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::SitDownAction,
		hubero_ros_msgs::SitDownActionFeedbackConstPtr,
		hubero_ros_msgs::SitDownActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_SIT_DOWN)
	);

	ac_sit_down_object_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::SitDownObjectAction,
		hubero_ros_msgs::SitDownObjectActionFeedbackConstPtr,
		hubero_ros_msgs::SitDownObjectActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_SIT_DOWN) + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX
	);

	ac_stand_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::StandAction,
		hubero_ros_msgs::StandActionFeedbackConstPtr,
		hubero_ros_msgs::StandActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_STAND)
	);

	ac_talk_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::TalkAction,
		hubero_ros_msgs::TalkActionFeedbackConstPtr,
		hubero_ros_msgs::TalkActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_TALK)
	);

	ac_talk_object_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::TalkObjectAction,
		hubero_ros_msgs::TalkObjectActionFeedbackConstPtr,
		hubero_ros_msgs::TalkObjectActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_TALK) + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX
	);

	ac_teleop_ptr_ = std::make_shared<ActionClient<
		hubero_ros_msgs::TeleopAction,
		hubero_ros_msgs::TeleopActionFeedbackConstPtr,
		hubero_ros_msgs::TeleopActionResultConstPtr>
		>(node_ptr_, actor_task_ns + TaskRequestBase::getTaskName(TASK_TELEOP)
	);
}

bool TaskRequestRosApi::followObject(const std::string& object_name) {
	hubero_ros_msgs::FollowObjectGoal action_goal;
	action_goal.object_name = object_name;
	followObject(action_goal);
}

bool TaskRequestRosApi::followObject(const hubero_ros_msgs::FollowObjectGoal& goal) {
	ac_follow_object_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::lieDown(const Vector3& pos, const double& yaw, const std::string& frame_id) {
	hubero_ros_msgs::LieDownGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.yaw = yaw;
	action_goal.frame = frame_id;
	lieDown(action_goal);
}

bool TaskRequestRosApi::lieDown(const hubero_ros_msgs::LieDownGoal& goal) {
	ac_lie_down_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::lieDownObject(const std::string& object_name, const double& height, const double& yaw) {
	hubero_ros_msgs::LieDownObjectGoal action_goal;
	action_goal.object_name = object_name;
	action_goal.height = height;
	action_goal.yaw = yaw;
	lieDownObject(action_goal);
}

bool TaskRequestRosApi::lieDownObject(const hubero_ros_msgs::LieDownObjectGoal& goal) {
	ac_lie_down_object_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::moveAround() {
	hubero_ros_msgs::MoveAroundGoal action_goal;
	moveAround(action_goal);
}

bool TaskRequestRosApi::moveAround(const hubero_ros_msgs::MoveAroundGoal& goal) {
	ac_move_around_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::moveToGoal(const Vector3& pos, const std::string& frame_id) {
	hubero_ros_msgs::MoveToGoalGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.frame = frame_id;
	moveToGoal(action_goal);
}

bool TaskRequestRosApi::moveToGoal(const hubero_ros_msgs::MoveToGoalGoal& goal) {
	ac_move_to_goal_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::moveToObject(const std::string& object_name) {
	hubero_ros_msgs::MoveToObjectGoal action_goal;
	action_goal.object_name = object_name;
	moveToObject(action_goal);
}

bool TaskRequestRosApi::moveToObject(const hubero_ros_msgs::MoveToObjectGoal& goal) {
	ac_move_to_object_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::run(const Vector3& pos, const std::string& frame_id) {
	hubero_ros_msgs::RunGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.frame = frame_id;
	run(action_goal);
}

bool TaskRequestRosApi::run(const hubero_ros_msgs::RunGoal& goal) {
	ac_run_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::sitDown(const Vector3& pos, const double& yaw, const std::string& frame_id) {
	hubero_ros_msgs::SitDownGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.yaw = yaw;
	action_goal.frame = frame_id;
	sitDown(action_goal);
}

bool TaskRequestRosApi::sitDown(const hubero_ros_msgs::SitDownGoal& goal) {
	ac_sit_down_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::sitDownObject(const std::string& object_name, const double& height, const double& yaw) {
	hubero_ros_msgs::SitDownObjectGoal action_goal;
	action_goal.object_name = object_name;
	action_goal.height = height;
	action_goal.yaw = yaw;
	sitDownObject(action_goal);
}

bool TaskRequestRosApi::sitDownObject(const hubero_ros_msgs::SitDownObjectGoal& goal) {
	ac_sit_down_object_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::stand() {
	hubero_ros_msgs::StandGoal action_goal;
	stand(action_goal);
}

bool TaskRequestRosApi::stand(const hubero_ros_msgs::StandGoal& goal) {
	ac_stand_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::talk(const Vector3& pos, const std::string& frame_id) {
	hubero_ros_msgs::TalkGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.frame = frame_id;
	talk(action_goal);
}

bool TaskRequestRosApi::talk(const hubero_ros_msgs::TalkGoal& goal) {
	ac_talk_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::talkObject(const std::string& object_name) {
	hubero_ros_msgs::TalkObjectGoal action_goal;
	action_goal.object_name = object_name;
	talkObject(action_goal);
}

bool TaskRequestRosApi::talkObject(const hubero_ros_msgs::TalkObjectGoal& goal) {
	ac_talk_object_ptr_->sendGoal(goal);
}

bool TaskRequestRosApi::teleop() {
	hubero_ros_msgs::TeleopGoal action_goal;
	teleop(action_goal);
}

bool TaskRequestRosApi::teleop(const hubero_ros_msgs::TeleopGoal& goal) {
	ac_teleop_ptr_->sendGoal(goal);
}

} // namespace hubero
