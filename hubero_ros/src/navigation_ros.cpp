#include <hubero_ros/navigation_ros.h>

#include <hubero_ros/utils/converter.h>
#include <hubero_common/logger.h>

#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

namespace hubero {

const int NavigationRos::SUBSCRIBER_QUEUE_SIZE = 10;
const int NavigationRos::PUBLISHER_QUEUE_SIZE = 15;

NavigationRos::NavigationRos(): NavigationBase::NavigationBase() {}

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

	// nav config
	std::string param_nav_get_plan_tolerance;
	nh.searchParam("/hubero_ros/navigation/nav_get_plan_tolerance", param_nav_get_plan_tolerance);
	nh.param(param_nav_get_plan_tolerance, nav_get_plan_tolerance_, 1.0);

	// prepare name
	std::string ns_nav = actor_name_ + "/";

	// initialize publishers, service clients, subscribers
	pub_mb_goal_ = node_ptr->getNodeHandlePtr()->advertise<move_base_msgs::MoveBaseActionGoal>(
		ns_nav + topic_nav_goal,
		PUBLISHER_QUEUE_SIZE
	);

	pub_mb_cancel_ = node_ptr->getNodeHandlePtr()->advertise<actionlib_msgs::GoalID>(
		ns_nav + topic_nav_cancel,
		PUBLISHER_QUEUE_SIZE
	);

	srv_mb_get_plan_ = node_ptr->getNodeHandlePtr()->serviceClient<nav_msgs::GetPlan>(
		ns_nav + srv_nav_get_plan
	);

	sub_cmd_vel_ = node_ptr->getNodeHandlePtr()->subscribe(
		ns_nav + topic_nav_cmd,
		SUBSCRIBER_QUEUE_SIZE,
		&NavigationRos::callbackCmdVel,
		this
	);

	pub_odom_ = node_ptr->getNodeHandlePtr()->advertise<nav_msgs::Odometry>(
		ns_nav + topic_nav_odometry,
		PUBLISHER_QUEUE_SIZE
	);

	HUBERO_LOG("[%s].[NavigationRos] Initialized ROS interface\r\n"
		"\t(pub) goal request topic  at '%s'\r\n"
		"\t(pub) goal cancel  topic  at '%s'\r\n"
		"\t(srv) get plan     client at '%s'\r\n"
		"\t(sub) cmd vel      topic  at '%s'\r\n",
		actor_name.c_str(),
		(node_ptr->getNamespaceName() + "/" + ns_nav + topic_nav_goal).c_str(),
		(node_ptr->getNamespaceName() + "/" + ns_nav + topic_nav_cancel).c_str(),
		(node_ptr->getNamespaceName() + "/" + ns_nav + srv_nav_get_plan).c_str(),
		(node_ptr->getNamespaceName() + "/" + ns_nav + topic_nav_cmd).c_str()
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

	NavigationBase::update(pose);

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

	// publish actor TF
	geometry_msgs::TransformStamped transform_actor {};
	transform_actor.header.stamp = ros::Time::now();
	transform_actor.header.frame_id = world_frame_name_;
	transform_actor.child_frame_id = actor_name_ + "/" + frame_base_;
	transform_actor.transform = ignPoseToMsgTf(pose);
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

void NavigationRos::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
	const std::lock_guard<std::mutex> lock(mutex_callback_);
	cmd_vel_.X(msg->linear.x);
	cmd_vel_.Y(msg->linear.y);
	cmd_vel_.Z(msg->angular.z);
}

// static
void NavigationRos::setIdealCovariance(boost::array<double, 36>& cov) {
	// update covariance - note that pose is perfectly known
	std::fill(cov.begin(), cov.end(), 0);
	// ones on diagonal
	for (unsigned int i = 0; i < cov.size(); i += 1 + sqrt(cov.size())) {
		cov[i] = 1.0;
	}
}

} // namespace hubero
