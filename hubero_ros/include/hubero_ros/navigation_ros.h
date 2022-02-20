#pragma once

#include <hubero_interfaces/navigation_base.h>
#include <hubero_ros/node.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include <memory>
#include <mutex>
#include <tuple>

namespace hubero {

/**
 * @brief Implements ROS interface for navigation tasks realization in HuBeRo
 *
 * @details @ref move_base is used as a planning interface
 * Topics configuration (for costmap generation) is placed inside YAML files.
 */
class NavigationRos: public NavigationBase {
public:
	// TODO: would look cleaner with C++17 'static constexpr'
	/// Defines default size of the queue used for publisher
	static const int PUBLISHER_QUEUE_SIZE;

	/// Defines default size of the queue used for subscriber
	static const int SUBSCRIBER_QUEUE_SIZE;

	/**
	 * @brief Constructor
	 */
	NavigationRos();

	/**
	 * @brief Initializes internal components of the class
	 *
	 * @param node_ptr pointer to the HuBeRo-related ROS node
	 * @param agent_name name of the agent, used as namespace for topics and services
	 * @param world_frame_name name of the frame that simulator poses are expressed in
	 * @param pose_initial initial pose of the actor (after startup) - used for 'odometry' calculations
	 * @return true If initialized properly
	 * @return false If not initialized properly
	 */
	bool initialize(
		std::shared_ptr<Node> node_ptr,
		const std::string& actor_name,
		const std::string& world_frame_name,
		const Pose3& pose_initial
	);

	/**
	 * @brief Evaluates possibility of pose reachability via trying to plan the full path to the @ref goal
	 *
	 * @param start
	 * @param goal
	 * @param frame
	 * @return true
	 * @return false
	 */
	virtual bool isPoseAchievable(const Pose3& start, const Pose3& goal, const std::string& frame) override;

	/**
	 * @brief Set the pose (localisation)
	 *
	 * @param pose
	 * @param vel_lin
	 * @param vel_ang
	 */
	virtual void update(const Pose3& pose, const Vector3& vel_lin, const Vector3& vel_ang) override;

	/**
	 * @brief Set the goal
	 *
	 * @param pose
	 * @param frame
	 * @return true
	 * @return false
	 */
	virtual bool setGoal(const Pose3& pose, const std::string& frame) override;

	/**
	 * @brief Cancels current goal (one goal is allowed at once)
	 *
	 * @return true
	 * @return false
	 */
	virtual bool cancelGoal() override;

	/**
	 * @brief Computes reachable pose that is closest to the given pose, starting from current pose from update call
	 */
	virtual Pose3 computeClosestAchievablePose(const Pose3& pose, const std::string& frame) override;

	/**
	 * @brief Get the velocity command
	 *
	 * @return Vector3
	 */
	virtual Vector3 getVelocityCmd() const override;

	/**
	 * @brief Sets ideal covariance in the @ref cov array
	 * @details boost array used here to comply with ROS messages
	 */
	static void setIdealCovariance(boost::array<double, 36>& cov);

	/**
	 * @brief Transforms actionlib_msgs::GoalStatus enum to hubero::TaskFeedbackType enum
	 */
	static TaskFeedbackType convertActionStatusToTaskFeedback(const uint8_t& status);

	/**
	 * @brief Transforms actionlib::SimpleClientGoalState::StateEnum enum to hubero::TaskFeedbackType enum
	 */
	static TaskFeedbackType convertSimpleClientStateToTaskFeedback(const uint8_t& status);

protected:
	/**
	 * @brief Callback for velocity command retrieval
	 */
	void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

	/**
	 * @defgroup mbinterfacetopic ROS move_base topic interface callbacks
	 * @{
	 */
	/**
	 * @brief Callback for movement action result retrieval
	 */
	void callbackFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);

	/**
	 * @brief Callback for movement action result retrieval
	 */
	void callbackResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
	/// @}

	/**
	 * @defgroup mbinterfaceaction ROS move_base action interface callbacks
	 * @{
	 */
	/**
	 * @brief Called when action is done
	 */
	void callbackActionDone(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr& msg
	);
	/// @}

	/**
	 * @brief Finds transform between coordinate systems using ROS TF buffer
	 * @return std::tuple<bool, Pose3> first element is true if transform (second elem) is valid
	 */
	std::tuple<bool, Pose3> findTransform(const std::string& frame_source, const std::string& frame_target) const;

	/**
	 * @brief Computes plan from start to goal using ROS service call
	 */
	nav_msgs::Path computePlan(
		const Pose3& start_pose,
		const std::string& start_frame,
		const Pose3& goal_pose,
		const std::string& goal_frame
	);

	/**
	 * @defgroup rosinterface ROS interface
	 * @{
	 */
	ros::Subscriber sub_cmd_vel_;
	ros::ServiceClient srv_mb_get_plan_;
	ros::Publisher pub_odom_;
	/// @brief Subscriber of the move_base simple action server's feedback topic
	ros::Subscriber sub_feedback_;
	/// @brief Subscriber of the move_base simple action server's result topic
	ros::Subscriber sub_result_;

	/// Helper typedefs
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;
	typedef std::shared_ptr<MoveBaseActionClient> MoveBaseActionClientPtr;

	/// @brief actionlib client of ROS move_base action server
	MoveBaseActionClientPtr nav_action_client_ptr_;
	/// @}

	/**
	 * @defgroup rosinterfacehelpers ROS Interface helpers
	 * @{
	 */
	/// @brief Flag that turns true when connection with action server was detected
	bool nav_action_server_connected_;

	/// @brief Flag that turns true when connection with service server was established
	bool nav_srv_mb_get_plan_exists_;

	/// @brief Stores most recent navigation goal, helps goal restoration
	move_base_msgs::MoveBaseGoal nav_goal_;
	/// @}

	/**
	 * @defgroup tf Transform frames
	 * @{
	 */
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	tf2_ros::TransformBroadcaster tf_broadcaster_;
	std::string frame_base_;
	std::string frame_global_ref_;
	std::string frame_local_ref_;
	std::string frame_laser_;
	std::string frame_camera_;
	/// @}

	/**
	 * @defgroup cmdvel Velocity commands
	 * @{
	 */
	Vector3 cmd_vel_;
	std::mutex mutex_callback_;
	/// @}

	/// @brief Initial pose of the actor - used for 'odometry' calculations
	Pose3 pose_initial_;

	/// @brief Tolerance (in meters) when requesting a path plan to a certain pose
	double nav_get_plan_tolerance_;
};

} // namespace hubero
