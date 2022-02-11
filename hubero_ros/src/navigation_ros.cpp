#include <hubero_ros/navigation_ros.h>

#include <hubero_ros/utils/converter.h>
#include <hubero_common/logger.h>

#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <thread>

namespace hubero {

const int NavigationRos::SUBSCRIBER_QUEUE_SIZE = 10;
const int NavigationRos::PUBLISHER_QUEUE_SIZE = 15;

NavigationRos::NavigationRos():
	NavigationBase::NavigationBase(),
	nav_action_server_connected_(false),
	nav_srv_mb_get_plan_exists_(false),
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

	if (!srv_mb_get_plan_.isValid()) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Navigation stack '%s' service is not valid\r\n",
			actor_name.c_str(),
			srv_nav_get_plan.c_str()
		);
	}

	HUBERO_LOG("[%s].[NavigationRos] Initialized ROS navigation stack interface\r\n"
		"\t(srv) get plan     client at '%s'\r\n"
		"\t(sub) cmd vel      topic  at '%s'\r\n"
		"\t(sub) feedback     topic  at '%s'\r\n"
		"\t(sub) result       topic  at '%s'\r\n",
		actor_name.c_str(),
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

	// action server connection - extensive conditions to print this only once
	if (
		nav_action_client_ptr_ != nullptr
		&& !nav_action_server_connected_
		&& nav_action_client_ptr_->isServerConnected()
	) {
		HUBERO_LOG("[%s].[NavigationRos] Connected to ROS navigation action server\r\n", actor_name_.c_str());
		nav_action_server_connected_ = true;
	}

	// service server connection
	if (!nav_srv_mb_get_plan_exists_ && srv_mb_get_plan_.exists()) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Navigation stack '%s' service finally exists\r\n",
			actor_name_.c_str(),
			srv_mb_get_plan_.getService().c_str()
		);
		nav_srv_mb_get_plan_exists_ = true;
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
	bool transform_valid = false;
	Pose3 global_ref_shift;
	std::tie(transform_valid, global_ref_shift) = findTransform(world_frame_name_, frame_global_ref_);

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

	/*
	 * It's ugly to start action client here, but it seems that move_base waits for odom msg to be received and then
	 * is ready to start. Trying to start the action client in @ref initialize freezes everything. This was helpful:
	 * https://answers.ros.org/question/345012/move_base-action-topics-exist-but-client-stuck-on-waitforserver/
	 */
	if (nav_action_client_ptr_ == nullptr) {
		// find action client namespace, based on e.g. odom topic
		auto action_ns = pub_odom_.getTopic();
		if (action_ns.back() == '/') {
			action_ns.pop_back();
		}
		auto action_ns_separator = action_ns.find_last_of("/");
		action_ns = action_ns.substr(0, action_ns_separator);

		nav_action_client_ptr_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(
			action_ns,
			true
		);
	}
}

bool NavigationRos::setGoal(const Pose3& pose, const std::string& frame) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return false;
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring 'setGoal' request\r\n",
			actor_name_.c_str()
		);
		return false;
	}

	NavigationBase::setGoal(pose, frame);

	auto time_current = ros::Time::now();
	actionlib_msgs::GoalID goal_id;
	goal_id.id = "";
	goal_id.stamp = time_current;

	nav_goal_.target_pose.header.frame_id = frame;
	nav_goal_.target_pose.header.stamp = ros::Time::now();
	nav_goal_.target_pose.pose = ignPoseToMsgPose(pose);

	// SimpleActiveCallback: we subscribe more extensive action state feedback
	// SimpleFeedbackCallback: feedback returns current pose of the 'base', which we know quite well
	nav_action_client_ptr_->sendGoal(
		nav_goal_,
		std::bind(&NavigationRos::callbackActionDone, this, std::placeholders::_1, std::placeholders::_2),
		MoveBaseActionClient::SimpleActiveCallback(),
		MoveBaseActionClient::SimpleFeedbackCallback()
	);

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

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring 'cancelGoal' request\r\n",
			actor_name_.c_str()
		);
		return false;
	}

	NavigationBase::cancelGoal();
	nav_action_client_ptr_->cancelGoal();
	return true;
}

Pose3 NavigationRos::computeClosestAchievablePose(const Pose3& pose, const std::string& frame) {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return pose;
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring request\r\n",
			actor_name_.c_str()
		);
		return pose;
	}

	// compute plan
	auto path = computePlan(current_pose_, world_frame_name_, pose, frame);

	// when goal pose is not reachable, then plan consists of set of valid poses + goal pose at the back of vector
	if (path.poses.size() >= 2) {
		// seconds to last element in a vector
		return msgPoseToIgnPose(path.poses.end()[-2].pose);
	} else if (path.poses.size() == 0) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Computed plan is empty - could not compute pose closest to given pose\r\n",
			actor_name_.c_str()
		);
		return pose;
	}

	HUBERO_LOG(
		"[%s].[NavigationRos] Computed plan is not empty (%d) but most likely contains something strange\r\n",
		actor_name_.c_str(),
		static_cast<int>(path.poses.size())
	);
	return pose;
}

Vector3 NavigationRos::getVelocityCmd() const {
	if (!isInitialized()) {
		HUBERO_LOG("[%s].[NavigationRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return Vector3();
	}

	if (!nav_action_server_connected_) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Did not manage to connect to ROS action server yet, ignoring 'getVelocityCmd'\r\n",
			actor_name_.c_str()
		);
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

// static
TaskFeedbackType NavigationRos::convertSimpleClientStateToTaskFeedback(const uint8_t& status) {
	switch (status) {
		case actionlib::SimpleClientGoalState::StateEnum::ABORTED:
			return TaskFeedbackType::TASK_FEEDBACK_ABORTED;
		case actionlib::SimpleClientGoalState::StateEnum::ACTIVE:
			return TaskFeedbackType::TASK_FEEDBACK_ACTIVE;
		case actionlib::SimpleClientGoalState::StateEnum::LOST:
			return TaskFeedbackType::TASK_FEEDBACK_LOST;
		case actionlib::SimpleClientGoalState::StateEnum::PENDING:
			return TaskFeedbackType::TASK_FEEDBACK_PENDING;
		case actionlib::SimpleClientGoalState::StateEnum::PREEMPTED:
			return TaskFeedbackType::TASK_FEEDBACK_PREEMPTED;
		case actionlib::SimpleClientGoalState::StateEnum::RECALLED:
			return TaskFeedbackType::TASK_FEEDBACK_RECALLED;
		case actionlib::SimpleClientGoalState::StateEnum::REJECTED:
			return TaskFeedbackType::TASK_FEEDBACK_REJECTED;
		case actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED:
			return TaskFeedbackType::TASK_FEEDBACK_SUCCEEDED;
		default:
			return TaskFeedbackType::TASK_FEEDBACK_UNDEFINED;
	}
}

void NavigationRos::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
	const std::lock_guard<std::mutex> lock(mutex_callback_);
	Vector3 cmd_vel_local;
	cmd_vel_local.X(msg->linear.x);
	cmd_vel_local.Y(msg->linear.y);
	cmd_vel_local.Z(msg->angular.z);
	cmd_vel_ = NavigationBase::convertCommandToGlobalCs(current_pose_.Rot().Yaw(), cmd_vel_local);
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

void NavigationRos::callbackActionDone(
	const actionlib::SimpleClientGoalState& state,
	const move_base_msgs::MoveBaseResultConstPtr& msg
) {
	auto fb_type = NavigationRos::convertSimpleClientStateToTaskFeedback(state.state_);
	if (fb_type == TASK_FEEDBACK_UNDEFINED) {
		return;
	}
	feedback_ = fb_type;
}

std::tuple<bool, Pose3> NavigationRos::findTransform(const std::string& frame_source, const std::string& frame_target) const {
	Pose3 transform;
	bool success = false;
	try {
		auto tf_msg = tf_buffer_.lookupTransform(frame_target, frame_source, ros::Time(0));
		transform = msgTfToPose(tf_msg.transform);
		success = true;
	} catch (tf2::TransformException& ex) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Could not transform '%s' to '%s' - exception: '%s'",
			actor_name_.c_str(),
			world_frame_name_.c_str(),
			frame_global_ref_.c_str(),
			ex.what()
		);
	}
	return std::make_tuple(success, transform);
}

nav_msgs::Path NavigationRos::computePlan(
	const Pose3& start_pose,
	const std::string& start_frame,
	const Pose3& goal_pose,
	const std::string& goal_frame
) {
	/*
	 * computePlan in a nutshell:
	 * store backup of the current goal, abort it, compute plan
	 * and restore the previous goal
	 */
	if (nav_action_client_ptr_->getState() == actionlib::SimpleClientGoalState::ACTIVE
		|| nav_action_client_ptr_->getState() == actionlib::SimpleClientGoalState::PENDING
	) {
		// this order allows to abort current goal and not update navigation status (callbacks detached)
		nav_action_client_ptr_->stopTrackingGoal();
		nav_action_client_ptr_->cancelAllGoals();
	}

	// service complains if start/goal pose is defined in frame other than the global reference frame
	bool transform_start_valid = false;
	Pose3 transform_start;
	std::tie(transform_start_valid, transform_start) = findTransform(start_frame, frame_global_ref_);

	bool transform_goal_valid = false;
	Pose3 transform_goal;
	std::tie(transform_goal_valid, transform_goal) = findTransform(goal_frame, frame_global_ref_);

	if (!transform_start_valid || !transform_goal_valid) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Could not compute valid plan because of TF lookup failure\r\n",
			actor_name_.c_str()
		);
		return nav_msgs::Path();
	}

	auto pose_start_global_ref = start_pose + transform_start;
	auto pose_goal_global_ref = goal_pose + transform_goal;

	// "planarize" poses
	auto pose_start_global_ref_plane = pose_start_global_ref;
	pose_start_global_ref_plane.Pos().Z(0.0);

	auto pose_goal_global_ref_plane = pose_goal_global_ref;
	pose_goal_global_ref_plane.Pos().Z(0.0);

	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response resp;
	auto ts = ros::Time::now();

	req.start.header.frame_id = frame_global_ref_;
	req.start.header.stamp = ts;
	req.start.header.seq = 0;
	req.start.pose = ignPoseToMsgPose(pose_start_global_ref_plane);

	req.goal.header.frame_id = frame_global_ref_;
	req.goal.header.stamp = ts;
	req.goal.header.seq = 0;
	req.goal.pose = ignPoseToMsgPose(pose_goal_global_ref_plane);
	req.tolerance = nav_get_plan_tolerance_;

	/*
	 * wait for action client to become free (LOST -> after cancel) to ask for a plan, ROS ERROR:
	 * move_base must be in an inactive state to make a plan for an external user
	 */
	while (nav_action_client_ptr_->getState() != actionlib::SimpleClientGoalState::LOST) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Waiting for navigation action client to compute a plan\r\n",
			actor_name_.c_str(),
		);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	/*
	 * // FIXME: process this call in separate thread and set goal according to the generated plan
	 * sometimes 1 sec of delay is not enough to force plan computation;
	 * Experiments show that the plan will be generated in first srv call or during next iteration
	 */
	bool success = false;
	for (unsigned int i = 0; i < 25; i++) {
		success = srv_mb_get_plan_.call(req, resp);
		if (success) {
			break;
		} else if (i > 0) {
			HUBERO_LOG(
				"[%s].[NavigationRos] Retrying to compute a valid plan for a %d time\r\n",
				actor_name_.c_str(),
				i + 1
			);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	// restore previous goal
	nav_action_client_ptr_->sendGoal(
		nav_goal_,
		std::bind(&NavigationRos::callbackActionDone, this, std::placeholders::_1, std::placeholders::_2),
		MoveBaseActionClient::SimpleActiveCallback(),
		MoveBaseActionClient::SimpleFeedbackCallback()
	);

	if (!success || resp.plan.poses.size() == 0) {
		HUBERO_LOG(
			"[%s].[NavigationRos] Couldn't find a plan from {x %2.1f, y %2.1f} to {x %2.1f, y %2.1f} despite tolerance of %2.4f\r\n",
			actor_name_.c_str(),
			start_pose.Pos().X(),
			start_pose.Pos().Y(),
			goal_pose.Pos().X(),
			goal_pose.Pos().Y(),
			nav_get_plan_tolerance_
		);
		return nav_msgs::Path();
	}
	return resp.plan;
}

} // namespace hubero
