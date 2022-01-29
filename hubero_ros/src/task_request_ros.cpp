#include <hubero_ros/task_request_ros.h>
#include <hubero_ros/utils/converter.h>

namespace hubero {

const std::string TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX = "_name";
const std::chrono::milliseconds TaskRequestRos::TASK_FEEDBACK_PERIOD = std::chrono::milliseconds(500);

TaskRequestRos::TaskRequestRos():
	TaskRequestBase::TaskRequestBase(),
	tf_listener_(tf_buffer_) {}

void TaskRequestRos::initialize(
	std::shared_ptr<Node> node_ptr,
	const std::string& actor_name,
	const std::string& world_frame_name
) {
	if (node_ptr == nullptr) {
		HUBERO_LOG("[TaskRequestRos] Pointer to Node is null, action servers will not be started\r\n");
		return;
	}

	actor_name_ = actor_name;
	world_frame_name_ = world_frame_name;

	// convert task IDs to task names
	auto name_task_stand = TaskRequestBase::getTaskName(TASK_STAND);
	auto name_task_move_to_goal = TaskRequestBase::getTaskName(TASK_MOVE_TO_GOAL);
	auto name_task_move_around = TaskRequestBase::getTaskName(TASK_MOVE_AROUND);
	auto name_task_lie_down = TaskRequestBase::getTaskName(TASK_LIE_DOWN);
	auto name_task_sit_down = TaskRequestBase::getTaskName(TASK_SIT_DOWN);
	auto name_task_follow_object = TaskRequestBase::getTaskName(TASK_FOLLOW_OBJECT);
	auto name_task_teleop = TaskRequestBase::getTaskName(TASK_TELEOP);
	auto name_task_run = TaskRequestBase::getTaskName(TASK_RUN);
	auto name_task_talk = TaskRequestBase::getTaskName(TASK_TALK);

	// prepare task namespace
	std::string task_ns = actor_name + "/" + node_ptr->getTaskNamespaceName() + "/";

	// create action servers
	as_follow_object_ = std::make_shared<ActionServer<hubero_ros_msgs::FollowObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_follow_object,
		std::bind(&TaskRequestRos::actionCbFollowObject, this, std::placeholders::_1),
		false
	);
	as_follow_object_->start();

	as_lie_down_ = std::make_shared<ActionServer<hubero_ros_msgs::LieDownAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_lie_down,
		std::bind(&TaskRequestRos::actionCbLieDown, this, std::placeholders::_1),
		false
	);
	as_lie_down_->start();

	as_lie_down_object_ = std::make_shared<ActionServer<hubero_ros_msgs::LieDownObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_lie_down + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
		std::bind(&TaskRequestRos::actionCbLieDownObject, this, std::placeholders::_1),
		false
	);
	as_lie_down_object_->start();

	as_move_around_ = std::make_shared<ActionServer<hubero_ros_msgs::MoveAroundAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_move_around,
		std::bind(&TaskRequestRos::actionCbMoveAround, this, std::placeholders::_1),
		false
	);
	as_move_around_->start();

	as_move_to_goal_ = std::make_shared<ActionServer<hubero_ros_msgs::MoveToGoalAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_move_to_goal,
		std::bind(&TaskRequestRos::actionCbMoveToGoal, this, std::placeholders::_1),
		false
	);
	as_move_to_goal_->start();

	as_move_to_object_ = std::make_shared<ActionServer<hubero_ros_msgs::MoveToObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_move_to_goal + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
		std::bind(&TaskRequestRos::actionCbMoveToObject, this, std::placeholders::_1),
		false
	);
	as_move_to_object_->start();

	as_run_ = std::make_shared<ActionServer<hubero_ros_msgs::RunAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_run,
		std::bind(&TaskRequestRos::actionCbRun, this, std::placeholders::_1),
		false
	);
	as_run_->start();

	as_sit_down_ = std::make_shared<ActionServer<hubero_ros_msgs::SitDownAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_sit_down,
		std::bind(&TaskRequestRos::actionCbSitDown, this, std::placeholders::_1),
		false
	);
	as_sit_down_->start();

	as_sit_down_object_ = std::make_shared<ActionServer<hubero_ros_msgs::SitDownObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_sit_down + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
		std::bind(&TaskRequestRos::actionCbSitDownObject, this, std::placeholders::_1),
		false
	);
	as_sit_down_object_->start();

	as_stand_ = std::make_shared<ActionServer<hubero_ros_msgs::StandAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_stand,
		std::bind(&TaskRequestRos::actionCbStand, this, std::placeholders::_1),
		false
	);
	as_stand_->start();

	as_talk_ = std::make_shared<ActionServer<hubero_ros_msgs::TalkAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_talk,
		std::bind(&TaskRequestRos::actionCbTalk, this, std::placeholders::_1),
		false
	);
	as_talk_->start();

	as_talk_object_ = std::make_shared<ActionServer<hubero_ros_msgs::TalkObjectAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_talk + TaskRequestRos::OBJECT_ORIENTED_TASK_SUFFIX,
		std::bind(&TaskRequestRos::actionCbTalkObject, this, std::placeholders::_1),
		false
	);
	as_talk_object_->start();

	as_teleop_ = std::make_shared<ActionServer<hubero_ros_msgs::TeleopAction>>(
		*node_ptr->getNodeHandlePtr(),
		task_ns + name_task_teleop,
		std::bind(&TaskRequestRos::actionCbTeleop, this, std::placeholders::_1),
		false
	);
	as_teleop_->start();
}

/**
 * @note Use of tf_buffer_.transform will produce linking errors in packages that use task request library.
 * Suggestions like this: https://answers.ros.org/question/261419/tf2-transformpose-in-c/ did not help
 */
Vector3 TaskRequestRos::transformToWorldFrame(const Vector3& pos, const std::string& frame_id) const {
	Pose3 tf_goal_to_world = computeTransformToWorld(Pose3(pos, Quaternion()), frame_id);
	Pose3 pos_world = Pose3(pos, Quaternion()) + tf_goal_to_world;
	return pos_world.Pos();
}

/**
 * @note Use of tf_buffer_.transform will produce linking errors in packages that use task request library
 */
Pose3 TaskRequestRos::transformToWorldFrame(const Pose3& pose, const std::string& frame_id) const {
	Pose3 tf_goal_to_world = computeTransformToWorld(pose, frame_id);
	Pose3 pos_world = pose + tf_goal_to_world;
	return pos_world;
}

Pose3 TaskRequestRos::computeTransformToWorld(const Pose3& pose, const std::string& frame_id) const {
	Pose3 tf_goal_to_world;
	try {
		auto tf_goal_world_msg = tf_buffer_.lookupTransform(world_frame_name_, frame_id, ros::Time(0));
		tf_goal_to_world = msgTfToPose(tf_goal_world_msg.transform);
	} catch (tf2::TransformException& ex) {
		HUBERO_LOG(
			"[%s].[TaskRequestRos] Could not transform '%s' to '%s' - exception: '%s'",
			actor_name_.c_str(),
			world_frame_name_.c_str(),
			frame_id.c_str(),
			ex.what()
		);
	}
	return tf_goal_to_world;
}

void TaskRequestRos::actionCbFollowObject(const hubero_ros_msgs::FollowObjectGoalConstPtr& goal) {
	// request task
	bool request_processed_ok = request(TASK_FOLLOW_OBJECT, goal->object_name);
	actionCbHandler<hubero_ros_msgs::FollowObjectResult, hubero_ros_msgs::FollowObjectFeedback>(
		request_processed_ok,
		TASK_FOLLOW_OBJECT,
		as_follow_object_
	);
}

void TaskRequestRos::actionCbLieDown(const hubero_ros_msgs::LieDownGoalConstPtr& goal) {
	// task objective preprocessing
	Pose3 goal_pose(msgPointToIgnVector(goal->pos), Quaternion(0.0, 0.0, goal->yaw));
	Pose3 goal_world_pose = transformToWorldFrame(goal_pose, goal->frame);
	// request task
	bool request_processed_ok = request(TASK_LIE_DOWN, goal_world_pose.Pos(), goal_world_pose.Rot().Yaw());
	actionCbHandler<hubero_ros_msgs::LieDownResult, hubero_ros_msgs::LieDownFeedback>(
		request_processed_ok,
		TASK_LIE_DOWN,
		as_lie_down_
	);
}

void TaskRequestRos::actionCbLieDownObject(const hubero_ros_msgs::LieDownObjectGoalConstPtr& goal) {
	HUBERO_LOG("[CAUTION] LieDownObject task is not supported yet, use plain LieDown instead\r\n");
	Pose3 goal_rel_world_pose(Vector3(0.0, 0.0, goal->height), Quaternion(0.0, 0.0, goal->yaw));
	// request task
	bool request_processed_ok = false; // request(TASK_LIE_DOWN, goal->...);
	actionCbHandler<hubero_ros_msgs::LieDownObjectResult, hubero_ros_msgs::LieDownObjectFeedback>(
		request_processed_ok,
		TASK_LIE_DOWN,
		as_lie_down_object_
	);
}

void TaskRequestRos::actionCbMoveAround(const hubero_ros_msgs::MoveAroundGoalConstPtr& goal) {
	// request task
	bool request_processed_ok = request(TASK_MOVE_AROUND);
	actionCbHandler<hubero_ros_msgs::MoveAroundResult, hubero_ros_msgs::MoveAroundFeedback>(
		request_processed_ok,
		TASK_MOVE_AROUND,
		as_move_around_
	);
}

void TaskRequestRos::actionCbMoveToGoal(const hubero_ros_msgs::MoveToGoalGoalConstPtr& goal) {
	// task objective preprocessing
	Pose3 goal_pose(msgPointToIgnVector(goal->pos), Quaternion());
	Pose3 goal_world_pose = transformToWorldFrame(goal_pose, goal->frame);
	// request task
	bool request_processed_ok = request(TASK_MOVE_TO_GOAL, goal_world_pose);
	actionCbHandler<hubero_ros_msgs::MoveToGoalResult, hubero_ros_msgs::MoveToGoalFeedback>(
		request_processed_ok,
		TASK_MOVE_TO_GOAL,
		as_move_to_goal_
	);
}

void TaskRequestRos::actionCbMoveToObject(const hubero_ros_msgs::MoveToObjectGoalConstPtr& goal) {
	HUBERO_LOG("[CAUTION] MoveToObject task is not supported yet, use plain MoveToGoal instead\r\n");
	bool request_processed_ok = false; // request(TASK_MOVE_TO_GOAL, goal->...);
	actionCbHandler<hubero_ros_msgs::MoveToObjectResult, hubero_ros_msgs::MoveToObjectFeedback>(
		request_processed_ok,
		TASK_MOVE_TO_GOAL,
		as_move_to_object_
	);
}

void TaskRequestRos::actionCbRun(const hubero_ros_msgs::RunGoalConstPtr& goal) {
	// task objective preprocessing
	Pose3 goal_pose(msgPointToIgnVector(goal->pos), Quaternion());
	Pose3 goal_world_pose = transformToWorldFrame(goal_pose, goal->frame);
	// request task
	bool request_processed_ok = request(TASK_RUN, goal_world_pose);
	actionCbHandler<hubero_ros_msgs::RunResult, hubero_ros_msgs::RunFeedback>(
		request_processed_ok,
		TASK_RUN,
		as_run_
	);
}

void TaskRequestRos::actionCbSitDown(const hubero_ros_msgs::SitDownGoalConstPtr& goal) {
	// task objective preprocessing
	Pose3 goal_pose(msgPointToIgnVector(goal->pos), Quaternion(0.0, 0.0, goal->yaw));
	Pose3 goal_world_pose = transformToWorldFrame(goal_pose, goal->frame);
	// request task
	bool request_processed_ok = request(TASK_SIT_DOWN, goal_world_pose.Pos(), goal_world_pose.Rot().Yaw());
	actionCbHandler<hubero_ros_msgs::SitDownResult, hubero_ros_msgs::SitDownFeedback>(
		request_processed_ok,
		TASK_SIT_DOWN,
		as_sit_down_
	);
}

void TaskRequestRos::actionCbSitDownObject(const hubero_ros_msgs::SitDownObjectGoalConstPtr& goal) {
	HUBERO_LOG("[CAUTION] SitDownObject task is not supported yet, use plain SitDown instead\r\n");
	// task objective preprocessing
	Pose3 goal_world_rel_pose(Vector3(0.0, 0.0, goal->height), Quaternion(0.0, 0.0, goal->yaw));
	// request task
	bool request_processed_ok = false; // request(TASK_SIT_DOWN, goal->...);
	actionCbHandler<hubero_ros_msgs::SitDownObjectResult, hubero_ros_msgs::SitDownObjectFeedback>(
		request_processed_ok,
		TASK_SIT_DOWN,
		as_sit_down_object_
	);
}

void TaskRequestRos::actionCbStand(const hubero_ros_msgs::StandGoalConstPtr& goal) {
	// request task
	bool request_processed_ok = request(TASK_STAND);
	actionCbHandler<hubero_ros_msgs::StandResult, hubero_ros_msgs::StandFeedback>(
		request_processed_ok,
		TASK_STAND,
		as_stand_
	);
}

void TaskRequestRos::actionCbTalk(const hubero_ros_msgs::TalkGoalConstPtr& goal) {
	// task objective preprocessing
	Pose3 goal_pose(msgPointToIgnVector(goal->pos), Quaternion());
	Pose3 goal_world_pose = transformToWorldFrame(goal_pose, goal->frame);
	// request task
	bool request_processed_ok = request(TASK_TALK, goal_world_pose);
	actionCbHandler<hubero_ros_msgs::TalkResult, hubero_ros_msgs::TalkFeedback>(
		request_processed_ok,
		TASK_TALK,
		as_talk_
	);
}

void TaskRequestRos::actionCbTalkObject(const hubero_ros_msgs::TalkObjectGoalConstPtr& goal) {
	HUBERO_LOG("[CAUTION] TalkObject task is not supported yet, use plain Talk instead\r\n");
	// request task
	bool request_processed_ok = false; // request(TASK_TALK, goal->...);
	actionCbHandler<hubero_ros_msgs::TalkObjectResult, hubero_ros_msgs::TalkObjectFeedback>(
		request_processed_ok,
		TASK_TALK,
		as_talk_object_
	);
}

void TaskRequestRos::actionCbTeleop(const hubero_ros_msgs::TeleopGoalConstPtr& goal) {
	// request task
	bool request_processed_ok = request(TASK_TELEOP);
	actionCbHandler<hubero_ros_msgs::TeleopResult, hubero_ros_msgs::TeleopFeedback>(
		request_processed_ok,
		TASK_TELEOP,
		as_teleop_
	);
}

} // namespace hubero
