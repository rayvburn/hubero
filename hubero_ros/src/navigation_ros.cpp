#include <hubero_ros/navigation_ros.h>

#include <hubero_ros/utils/converter.h>
#include <hubero_common/logger.h>

#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

namespace hubero {

const int NavigationRos::SUBSCRIBER_QUEUE_SIZE = 10;
const int NavigationRos::PUBLISHER_QUEUE_SIZE = 15;

NavigationRos::NavigationRos():
	NavigationBase::NavigationBase(),
	tf_listener_(tf_buffer_) {}

bool NavigationRos::initialize(
	std::shared_ptr<Node> node_ptr,
	const std::string& actor_name,
	const std::string& world_frame_name,
	const Pose3& pose_initial
) {
	if (this->isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Already initialized, aborting\r\n", actor_name.c_str());
		return false;
	}

	// initialize base class after objects from args are copied
	pose_initial_ = pose_initial;
	NavigationBase::initialize(actor_name, world_frame_name);

	// helps loading params (looks from a global scope)
	ros::NodeHandle nh;

	// frames
	std::string frame_base_param;
	nh.searchParam("/hubero_ros/actor_frames/base", frame_base_param);
	nh.param(frame_base_param, frame_base_, std::string("base_footprint"));

	std::string frame_global_ref_param;
	nh.searchParam("/hubero_ros/actor_frames/global_ref", frame_global_ref_param);
	nh.param(frame_global_ref_param, frame_global_ref_, std::string("map"));

	std::string frame_local_ref_param;
	nh.searchParam("/hubero_ros/actor_frames/local_ref", frame_local_ref_param);
	nh.param(frame_local_ref_param, frame_local_ref_, std::string("odom"));

	std::string frame_laser_param;
	nh.searchParam("/hubero_ros/actor_frames/lidar", frame_laser_param);
	nh.param(frame_laser_param, frame_laser_, std::string("base_laser_link"));

	std::string frame_camera_param;
	nh.searchParam("/hubero_ros/actor_frames/camera", frame_camera_param);
	nh.param(frame_camera_param, frame_camera_, std::string("base_camera_link"));

	// nav topics
	std::string topic_nav_goal;
	nh.searchParam("/hubero_ros/navigation/goal_topic", topic_nav_goal);
	nh.param(topic_nav_goal, topic_nav_goal, std::string("nav/goal"));

	std::string topic_nav_cancel;
	nh.searchParam("/hubero_ros/navigation/cancel_topic", topic_nav_cancel);
	nh.param(topic_nav_cancel, topic_nav_cancel, std::string("nav/cancel"));

	std::string srv_nav_get_plan;
	nh.searchParam("/hubero_ros/navigation/get_plan_srv", srv_nav_get_plan);
	nh.param(srv_nav_get_plan, srv_nav_get_plan, std::string("nav/get_plan"));

	std::string topic_nav_cmd;
	nh.searchParam("/hubero_ros/navigation/command_topic", topic_nav_cmd);
	nh.param(topic_nav_cmd, topic_nav_cmd, std::string("nav/cmd_vel"));

	std::string topic_nav_odometry;
	nh.searchParam("/hubero_ros/navigation/odometry_topic", topic_nav_odometry);
	nh.param(topic_nav_odometry, topic_nav_odometry, std::string("nav/odom"));

	std::string topic_nav_feedback;
	nh.searchParam("/hubero_ros/navigation/feedback_topic", topic_nav_feedback);
	nh.param(topic_nav_feedback, topic_nav_feedback, std::string("nav/feedback"));

	std::string topic_nav_result;
	nh.searchParam("/hubero_ros/navigation/result_topic", topic_nav_result);
	nh.param(topic_nav_result, topic_nav_result, std::string("nav/result"));

	// nav config
	std::string param_nav_get_plan_tolerance;
	nh.searchParam("/hubero_ros/navigation/nav_get_plan_tolerance", param_nav_get_plan_tolerance);
	nh.param(param_nav_get_plan_tolerance, nav_get_plan_tolerance_, 1.0);

	// initialize publishers, service clients, subscribers
	pub_mb_goal_ = node_ptr->getNodeHandlePtr()->advertise<move_base_msgs::MoveBaseActionGoal>(
		topic_nav_goal,
		PUBLISHER_QUEUE_SIZE
	);

	pub_mb_cancel_ = node_ptr->getNodeHandlePtr()->advertise<actionlib_msgs::GoalID>(
		topic_nav_cancel,
		PUBLISHER_QUEUE_SIZE
	);

	srv_mb_get_plan_ = node_ptr->getNodeHandlePtr()->serviceClient<nav_msgs::GetPlan>(
		srv_nav_get_plan
	);

	sub_cmd_vel_ = node_ptr->getNodeHandlePtr()->subscribe(
		topic_nav_cmd,
		SUBSCRIBER_QUEUE_SIZE,
		&NavigationRos::callbackCmdVel,
		this
	);

	pub_odom_ = node_ptr->getNodeHandlePtr()->advertise<nav_msgs::Odometry>(
		topic_nav_odometry,
		PUBLISHER_QUEUE_SIZE
	);

	sub_feedback_ = node_ptr->getNodeHandlePtr()->subscribe(
		topic_nav_feedback,
		SUBSCRIBER_QUEUE_SIZE,
		&NavigationRos::callbackFeedback,
		this
	);

	sub_result_ = node_ptr->getNodeHandlePtr()->subscribe(
		topic_nav_result,
		SUBSCRIBER_QUEUE_SIZE,
		&NavigationRos::callbackResult,
		this
	);

	HUBERO_LOG("[%s].[NavigationRos] Initialized ROS navigation stack interface\r\n"
		"\t(pub) goal request topic  at '%s'\r\n"
		"\t(pub) goal cancel  topic  at '%s'\r\n"
		"\t(srv) get plan     client at '%s'\r\n"
		"\t(sub) cmd vel      topic  at '%s'\r\n"
		"\t(sub) feedback     topic  at '%s'\r\n"
		"\t(sub) result       topic  at '%s'\r\n",
		actor_name.c_str(),
		topic_nav_goal.c_str(),
		topic_nav_cancel.c_str(),
		srv_nav_get_plan.c_str(),
		topic_nav_cmd.c_str(),
		topic_nav_feedback.c_str(),
		topic_nav_result.c_str()
	);

	return true;
}

bool NavigationRos::isPoseAchievable(const Pose3& start, const Pose3& goal, const std::string& frame) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return false;
	}

	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response resp;
	req.start.header.frame_id = frame;
	req.start.header.stamp = ros::Time::now();
	req.start.pose = ignPoseToMsgPose(start);
	req.goal.header = req.start.header;
	req.goal.pose = ignPoseToMsgPose(goal);
	req.tolerance = nav_get_plan_tolerance_;
	bool success = srv_mb_get_plan_.call(req, resp);

	if (!success || resp.plan.poses.size() == 0) {
		return false;
	}
	// // TODO: verify
	return true;
}

void NavigationRos::update(const Pose3& pose, const Vector3& vel_lin, const Vector3& vel_ang) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return;
	}

	// do not call base class update - let Navigation stack take care about feedback update and goal reaching
	current_pose_ = pose;

	// publish odom
	nav_msgs::Odometry odometry {};
	// header
	odometry.header.stamp = ros::Time::now();
	odometry.header.frame_id = frame_local_ref_;
	odometry.child_frame_id = frame_base_;
	// pose
	Pose3 odom_pose = pose - pose_initial_;
	odometry.pose.pose = ignPoseToMsgPose(odom_pose);
	setIdealCovariance(odometry.pose.covariance);
	// twist
	odometry.twist.twist = ignVectorsToMsgTwist(vel_lin, vel_ang);
	setIdealCovariance(odometry.twist.covariance);
	pub_odom_.publish(odometry);

	/*
	 * Find TFs: world->map, map->odom to publish odometry etc.
	 * NOTE: this is a hack, compared to a typical approach with a real robot, but poses from simulator are expressed
	 * in a global, static frame, whereas we want to create a tree of transforms for the actor - odom->base_footprint.
	 * These computations aim to create a separate TF tree for each actor. All actor trees derive from the one common
	 * frame (typically - "world").
	 */
	Pose3 global_ref_shift;
	try {
		auto tf_world_global_ref = tf_buffer_.lookupTransform(frame_global_ref_, world_frame_name_, ros::Time(0));
		global_ref_shift = msgTfToPose(tf_world_global_ref.transform);
	} catch (tf2::TransformException& ex) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Could not transform '%s' to '%s' - exception: '%s'",
			actor_name_.c_str(),
			world_frame_name_.c_str(),
			frame_global_ref_.c_str(),
			ex.what()
		);
	}

	// publish odom TF
	geometry_msgs::TransformStamped transform_odom {};
	transform_odom.header.stamp = ros::Time::now();
	transform_odom.header.frame_id = frame_global_ref_;
	transform_odom.child_frame_id = frame_local_ref_;
	// ignore height of the pose in this step - this will produce odometry frame at the same height as global ref frame
	Pose3 pose_initial_plane(Vector3(pose_initial_.Pos().X(), pose_initial_.Pos().Y(), 0.0), pose_initial_.Rot());
	transform_odom.transform = ignPoseToMsgTf(pose_initial_plane - global_ref_shift);
	tf_broadcaster_.sendTransform(transform_odom);

	// publish actor base TF
	geometry_msgs::TransformStamped transform_actor {};
	transform_actor.header.stamp = ros::Time::now();
	transform_actor.header.frame_id = frame_local_ref_;
	transform_actor.child_frame_id = frame_base_;
	// elevate with pose_initial_.Pos().Z() that was ignored in the previous TF
	Pose3 pose_base(pose - pose_initial_);
	pose_base.Pos().Z() += pose_initial_.Pos().Z();
	transform_actor.transform = ignPoseToMsgTf(pose_base);
	tf_broadcaster_.sendTransform(transform_actor);
}

bool NavigationRos::setGoal(const Pose3& pose, const std::string& frame) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return false;
	}

	NavigationBase::setGoal(pose, frame);

	move_base_msgs::MoveBaseActionGoal goal;
	actionlib_msgs::GoalID goal_id;
	auto time_current = ros::Time::now();
	goal_id.id = "";
	goal_id.stamp = time_current;
	goal.goal_id = goal_id;
	goal.header.frame_id = frame;
	goal.goal.target_pose.header.frame_id = frame;
	goal.goal.target_pose.header.stamp = time_current;
	goal.goal.target_pose.pose = ignPoseToMsgPose(pose);
	pub_mb_goal_.publish(goal);
	HUBERO_LOG(
		"[%s].[NavigationRos] Trying to set goal: x %1.2f, y %1.2f, z %1.2f\r\n",
		actor_name_.c_str(),
		pose.Pos().X(),
		pose.Pos().Y(),
		pose.Pos().Z()
	);
	return true;
}

bool NavigationRos::cancelGoal() {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return false;
	}

	actionlib_msgs::GoalID goal_cancel;
	pub_mb_cancel_.publish(goal_cancel);
	HUBERO_LOG(
		"[%s].[NavigationRos] Trying to cancel goal with ID '%s'\r\n",
		actor_name_.c_str(),
		goal_cancel.id.c_str()
	);
	return true;
}

Vector3 NavigationRos::getVelocityCmd() const {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return Vector3();
	}
	return cmd_vel_;
}

// static
void NavigationRos::setIdealCovariance(boost::array<double, 36>& cov) {
	// update covariance - note that pose is perfectly known
	std::fill(cov.begin(), cov.end(), 0);
	// ones on diagonal
	for (unsigned int i = 0; i < cov.size(); i += 1 + sqrt(cov.size())) {
		cov[i] = 1e-06;
	}
}

// static
TaskFeedbackType NavigationRos::convertActionStatusToTaskFeedback(const uint8_t& status) {
	// NOTE: basically GoalStatus matches TaskFeedbackType, but this allows to maintain proper operation
	// even if API changes
	switch (status) {
		case actionlib_msgs::GoalStatus::ABORTED:
			return TaskFeedbackType::TASK_FEEDBACK_ABORTED;
		case actionlib_msgs::GoalStatus::ACTIVE:
			return TaskFeedbackType::TASK_FEEDBACK_ACTIVE;
		case actionlib_msgs::GoalStatus::LOST:
			return TaskFeedbackType::TASK_FEEDBACK_LOST;
		case actionlib_msgs::GoalStatus::PENDING:
			return TaskFeedbackType::TASK_FEEDBACK_PENDING;
		case actionlib_msgs::GoalStatus::PREEMPTED:
			return TaskFeedbackType::TASK_FEEDBACK_PREEMPTED;
		case actionlib_msgs::GoalStatus::PREEMPTING:
			return TaskFeedbackType::TASK_FEEDBACK_PREEMPTING;
		case actionlib_msgs::GoalStatus::RECALLED:
			return TaskFeedbackType::TASK_FEEDBACK_RECALLED;
		case actionlib_msgs::GoalStatus::RECALLING:
			return TaskFeedbackType::TASK_FEEDBACK_RECALLING;
		case actionlib_msgs::GoalStatus::REJECTED:
			return TaskFeedbackType::TASK_FEEDBACK_REJECTED;
		case actionlib_msgs::GoalStatus::SUCCEEDED:
			return TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED;
		default:
			return TaskFeedbackType::TASK_FEEDBACK_UNDEFINED;
	}
}

void NavigationRos::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
	const std::lock_guard<std::mutex> lock(mutex_callback_);
	cmd_vel_.X(msg->linear.x);
	cmd_vel_.Y(msg->linear.y);
	cmd_vel_.Z(msg->angular.z);
}

void NavigationRos::callbackFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
	const std::lock_guard<std::mutex> lock(mutex_callback_);
	auto fb_type = NavigationRos::convertActionStatusToTaskFeedback(msg->status.status);
	if (fb_type == TASK_FEEDBACK_UNDEFINED) {
		return;
	}
	feedback_ = fb_type;
}

void NavigationRos::callbackResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
	const std::lock_guard<std::mutex> lock(mutex_callback_);
	auto fb_type = NavigationRos::convertActionStatusToTaskFeedback(msg->status.status);
	if (fb_type == TASK_FEEDBACK_UNDEFINED) {
		return;
	}
	feedback_ = fb_type;
}

} // namespace hubero
