#include <hubero_ros/task_request_ros_api.h>
#include <hubero_ros/task_request_ros.h>
#include <hubero_ros/utils/converter.h>

#include <iostream>

namespace hubero {

TaskRequestRosApi::TaskRequestRosApi(const std::string& actor_name):
	node_ptr_(std::make_shared<Node>("task_request_ros_api_node"))
{
	// assignment - init list does not support string reference copying
	actor_name_ = actor_name;

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
	return followObject(action_goal);
}

bool TaskRequestRosApi::followObject(const hubero_ros_msgs::FollowObjectGoal& goal) {
	return sendActionGoal(ac_follow_object_ptr_, goal);
}

bool TaskRequestRosApi::stopFollowingObject() {
	return cancelActionGoals(ac_follow_object_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getFollowObjectState() const {
	return getActionStateType(ac_follow_object_ptr_);
}

std::string TaskRequestRosApi::getFollowObjectStateDescription() const {
	return getActionStateDescription(ac_follow_object_ptr_);
}

bool TaskRequestRosApi::lieDown(const Vector3& pos, const double& yaw, const std::string& frame_id) {
	hubero_ros_msgs::LieDownGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.yaw = yaw;
	action_goal.frame = frame_id;
	return lieDown(action_goal);
}

bool TaskRequestRosApi::lieDown(const hubero_ros_msgs::LieDownGoal& goal) {
	return sendActionGoal(ac_lie_down_ptr_, goal);
}

bool TaskRequestRosApi::lieDownObject(const std::string& object_name, const double& height, const double& yaw) {
	hubero_ros_msgs::LieDownObjectGoal action_goal;
	action_goal.object_name = object_name;
	action_goal.height = height;
	action_goal.yaw = yaw;
	return lieDownObject(action_goal);
}

bool TaskRequestRosApi::lieDownObject(const hubero_ros_msgs::LieDownObjectGoal& goal) {
	return sendActionGoal(ac_lie_down_object_ptr_, goal);
}

bool TaskRequestRosApi::stopLyingDown() {
	return cancelActionGoals(ac_lie_down_ptr_);
}

bool TaskRequestRosApi::stopLyingDownObject() {
	return cancelActionGoals(ac_lie_down_object_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getLieDownState() const {
	return getActionStateType(ac_lie_down_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getLieDownObjectState() const {
	return getActionStateType(ac_lie_down_object_ptr_);
}

std::string TaskRequestRosApi::getLieDownStateDescription() const {
	return getActionStateDescription(ac_lie_down_ptr_);
}

std::string TaskRequestRosApi::getLieDownObjectStateDescription() const {
	return getActionStateDescription(ac_lie_down_object_ptr_);
}

bool TaskRequestRosApi::moveAround() {
	hubero_ros_msgs::MoveAroundGoal action_goal;
	return moveAround(action_goal);
}

bool TaskRequestRosApi::moveAround(const hubero_ros_msgs::MoveAroundGoal& goal) {
	return sendActionGoal(ac_move_around_ptr_, goal);
}

bool TaskRequestRosApi::stopMovingAround() {
	return cancelActionGoals(ac_move_around_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getMoveAroundState() const {
	return getActionStateType(ac_move_around_ptr_);
}

std::string TaskRequestRosApi::getMoveAroundStateDescription() const {
	return getActionStateDescription(ac_move_around_ptr_);
}

bool TaskRequestRosApi::moveToGoal(const Vector3& pos, const double& yaw, const std::string& frame_id) {
	hubero_ros_msgs::MoveToGoalGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.yaw = yaw;
	action_goal.frame = frame_id;
	return moveToGoal(action_goal);
}

bool TaskRequestRosApi::moveToGoal(const hubero_ros_msgs::MoveToGoalGoal& goal) {
	return sendActionGoal(ac_move_to_goal_ptr_, goal);
}

bool TaskRequestRosApi::moveToObject(const std::string& object_name) {
	hubero_ros_msgs::MoveToObjectGoal action_goal;
	action_goal.object_name = object_name;
	return moveToObject(action_goal);
}

bool TaskRequestRosApi::moveToObject(const hubero_ros_msgs::MoveToObjectGoal& goal) {
	return sendActionGoal(ac_move_to_object_ptr_, goal);
}

bool TaskRequestRosApi::stopMovingToGoal() {
	return cancelActionGoals(ac_move_to_goal_ptr_);
}

bool TaskRequestRosApi::stopMovingToObject() {
	return cancelActionGoals(ac_move_to_object_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getMoveToGoalState() const {
	return getActionStateType(ac_move_to_goal_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getMoveToObjectState() const {
	return getActionStateType(ac_move_to_object_ptr_);
}

std::string TaskRequestRosApi::getMoveToGoalStateDescription() const {
	return getActionStateDescription(ac_move_to_goal_ptr_);
}

std::string TaskRequestRosApi::getMoveToObjectStateDescription() const {
	return getActionStateDescription(ac_move_to_object_ptr_);
}

bool TaskRequestRosApi::run(const Vector3& pos, const double& yaw, const std::string& frame_id) {
	hubero_ros_msgs::RunGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.yaw = yaw;
	action_goal.frame = frame_id;
	return run(action_goal);
}

bool TaskRequestRosApi::run(const hubero_ros_msgs::RunGoal& goal) {
	return sendActionGoal(ac_run_ptr_, goal);
}

bool TaskRequestRosApi::stopRunning() {
	return cancelActionGoals(ac_run_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getRunState() const {
	return getActionStateType(ac_run_ptr_);
}

std::string TaskRequestRosApi::getRunStateDescription() const {
	return getActionStateDescription(ac_run_ptr_);
}

bool TaskRequestRosApi::sitDown(const Vector3& pos, const double& yaw, const std::string& frame_id) {
	hubero_ros_msgs::SitDownGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.yaw = yaw;
	action_goal.frame = frame_id;
	return sitDown(action_goal);
}

bool TaskRequestRosApi::sitDown(const hubero_ros_msgs::SitDownGoal& goal) {
	return sendActionGoal(ac_sit_down_ptr_, goal);
}

bool TaskRequestRosApi::sitDownObject(const std::string& object_name, const double& height, const double& yaw) {
	hubero_ros_msgs::SitDownObjectGoal action_goal;
	action_goal.object_name = object_name;
	action_goal.height = height;
	action_goal.yaw = yaw;
	return sitDownObject(action_goal);
}

bool TaskRequestRosApi::sitDownObject(const hubero_ros_msgs::SitDownObjectGoal& goal) {
	return sendActionGoal(ac_sit_down_object_ptr_, goal);
}

bool TaskRequestRosApi::stopSittingDown() {
	return cancelActionGoals(ac_sit_down_ptr_);
}

bool TaskRequestRosApi::stopSittingDownObject() {
	return cancelActionGoals(ac_sit_down_object_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getSitDownState() const {
	return getActionStateType(ac_sit_down_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getSitDownObjectState() const {
	return getActionStateType(ac_sit_down_object_ptr_);
}

std::string TaskRequestRosApi::getSitDownStateDescription() const {
	return getActionStateDescription(ac_sit_down_ptr_);
}

std::string TaskRequestRosApi::getSitDownObjectStateDescription() const {
	return getActionStateDescription(ac_sit_down_object_ptr_);
}

bool TaskRequestRosApi::stand() {
	hubero_ros_msgs::StandGoal action_goal;
	return stand(action_goal);
}

bool TaskRequestRosApi::stand(const hubero_ros_msgs::StandGoal& goal) {
	return sendActionGoal(ac_stand_ptr_, goal);
}

TaskFeedbackType TaskRequestRosApi::getStandState() const {
	return getActionStateType(ac_stand_ptr_);
}

std::string TaskRequestRosApi::getStandStateDescription() const {
	return getActionStateDescription(ac_stand_ptr_);
}

bool TaskRequestRosApi::talk(const Vector3& pos, const double& yaw, const std::string& frame_id) {
	hubero_ros_msgs::TalkGoal action_goal;
	action_goal.pos = ignVectorToMsgPoint(pos);
	action_goal.yaw = yaw;
	action_goal.frame = frame_id;
	return talk(action_goal);
}

bool TaskRequestRosApi::talk(const hubero_ros_msgs::TalkGoal& goal) {
	return sendActionGoal(ac_talk_ptr_, goal);
}

bool TaskRequestRosApi::talkObject(const std::string& object_name) {
	hubero_ros_msgs::TalkObjectGoal action_goal;
	action_goal.object_name = object_name;
	return talkObject(action_goal);
}

bool TaskRequestRosApi::talkObject(const hubero_ros_msgs::TalkObjectGoal& goal) {
	return sendActionGoal(ac_talk_object_ptr_, goal);
}

bool TaskRequestRosApi::stopTalking() {
	return cancelActionGoals(ac_talk_ptr_);
}

bool TaskRequestRosApi::stopTalkingObject() {
	return cancelActionGoals(ac_talk_object_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getTalkState() const {
	return getActionStateType(ac_talk_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getTalkObjectState() const {
	return getActionStateType(ac_talk_object_ptr_);
}

std::string TaskRequestRosApi::getTalkStateDescription() const {
	return getActionStateDescription(ac_talk_ptr_);
}

std::string TaskRequestRosApi::getTalkObjectStateDescription() const {
	return getActionStateDescription(ac_talk_object_ptr_);
}

bool TaskRequestRosApi::teleop() {
	hubero_ros_msgs::TeleopGoal action_goal;
	return teleop(action_goal);
}

bool TaskRequestRosApi::teleop(const hubero_ros_msgs::TeleopGoal& goal) {
	return sendActionGoal(ac_teleop_ptr_, goal);
}

bool TaskRequestRosApi::stopTeleop() {
	return cancelActionGoals(ac_teleop_ptr_);
}

TaskFeedbackType TaskRequestRosApi::getTeleopState() const {
	return getActionStateType(ac_teleop_ptr_);
}

std::string TaskRequestRosApi::getTeleopStateDescription() const {
	return getActionStateDescription(ac_teleop_ptr_);
}

} // namespace hubero
