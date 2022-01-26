#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_interfaces/task_request_base.h>
#include <hubero_ros/node.h>

#include <hubero_ros_msgs/FollowObjectAction.h>
#include <hubero_ros_msgs/LieDownAction.h>
#include <hubero_ros_msgs/LieDownObjectAction.h>
#include <hubero_ros_msgs/MoveAroundAction.h>
#include <hubero_ros_msgs/MoveToGoalAction.h>
#include <hubero_ros_msgs/MoveToObjectAction.h>
#include <hubero_ros_msgs/RunAction.h>
#include <hubero_ros_msgs/SitDownAction.h>
#include <hubero_ros_msgs/SitDownObjectAction.h>
#include <hubero_ros_msgs/StandAction.h>
#include <hubero_ros_msgs/TalkAction.h>
#include <hubero_ros_msgs/TalkObjectAction.h>
#include <hubero_ros_msgs/TeleopAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalStatus.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace hubero {

/**
 * @brief Provides ROS interface to request tasks from actors and allows to monitor execution using feedback data
 */
class TaskRequestRos: public TaskRequestBase {
public:
	/// Alias declaration
	template <typename T>
	using ActionServer = actionlib::SimpleActionServer<T>;

	/// Alias declaration
	template <typename T>
	using ActionServerPtr = std::shared_ptr<ActionServer<T>>;

	// TODO: would look cleaner with C++17 'static constexpr'
	/// Contains suffix attached to basic action name, e.g., MoveToGoal vs MoveToObject
	static const std::string OBJECT_ORIENTED_TASK_SUFFIX;

	/// How often task feedback should be published
	static const std::chrono::milliseconds TASK_FEEDBACK_PERIOD;

	/**
	 * @brief Default constructor
	 */
	TaskRequestRos();

	/**
	 * @brief Initializes ROS interface to request tasks from simulated actor, named @ref actor_name
	 * @param world_frame_name ID of the coordinate system that actor operates in (i.e. simulator frame of reference)
	 */
	void initialize(
		std::shared_ptr<Node> node_ptr,
		const std::string& actor_name,
		const std::string& world_frame_name
	);

	/**
	 * @brief Returns transformed @ref pos vector
	 * @details @ref pos, expressed in @ref frame_id, is transformed to vector expressed in @ref world_frame_name_
	 */
	Vector3 transformToWorldFrame(const Vector3& pos, const std::string& frame_id) const;

	/**
	 * @brief Returns transformed @ref pose pose
	 * @details @ref pose, expressed in @ref frame_id, is transformed to pose expressed in @ref world_frame_name_
	 */
	Pose3 transformToWorldFrame(const Pose3& pose, const std::string& frame_id) const;

protected:
	/// @brief Name of the actor
	std::string actor_name_;

	/**
	 * @defgroup actions ROS actions
	 * @{
	 */
	ActionServerPtr<hubero_ros_msgs::FollowObjectAction> as_follow_object_;
	ActionServerPtr<hubero_ros_msgs::LieDownAction> as_lie_down_;
	ActionServerPtr<hubero_ros_msgs::LieDownObjectAction> as_lie_down_object_;
	ActionServerPtr<hubero_ros_msgs::MoveAroundAction> as_move_around_;
	ActionServerPtr<hubero_ros_msgs::MoveToGoalAction> as_move_to_goal_;
	ActionServerPtr<hubero_ros_msgs::MoveToObjectAction> as_move_to_object_;
	ActionServerPtr<hubero_ros_msgs::RunAction> as_run_;
	ActionServerPtr<hubero_ros_msgs::SitDownAction> as_sit_down_;
	ActionServerPtr<hubero_ros_msgs::SitDownObjectAction> as_sit_down_object_;
	ActionServerPtr<hubero_ros_msgs::StandAction> as_stand_;
	ActionServerPtr<hubero_ros_msgs::TalkAction> as_talk_;
	ActionServerPtr<hubero_ros_msgs::TalkObjectAction> as_talk_object_;
	ActionServerPtr<hubero_ros_msgs::TeleopAction> as_teleop_;
	/// @}

	/**
	 * @defgroup tf Frame transformations
	 * Used for position preparation, we aim core of HuBeRo to operate in simulated world's frame
	 * @{
	 */
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	/// @brief Name of the frame that incoming ( @ref update ) poses are referenced in
	std::string world_frame_name_;

	/**
	 * @brief Performs lookup in ROS TF buffer, looking for @ref frame_id -> @ref world_frame_name_ transform
	 */
	Pose3 computeTransformToWorld(const Pose3& pose, const std::string& frame_id) const;
	/// @}

	/**
	 * @defgroup actioncallbacks Callbacks for each action
	 * @{
	 */
	void actionCbFollowObject(const hubero_ros_msgs::FollowObjectGoalConstPtr& goal);
	void actionCbLieDown(const hubero_ros_msgs::LieDownGoalConstPtr& goal);
	void actionCbLieDownObject(const hubero_ros_msgs::LieDownObjectGoalConstPtr& goal);
	void actionCbMoveAround(const hubero_ros_msgs::MoveAroundGoalConstPtr& goal);
	void actionCbMoveToGoal(const hubero_ros_msgs::MoveToGoalGoalConstPtr& goal);
	void actionCbMoveToObject(const hubero_ros_msgs::MoveToObjectGoalConstPtr& goal);
	void actionCbRun(const hubero_ros_msgs::RunGoalConstPtr& goal);
	void actionCbSitDown(const hubero_ros_msgs::SitDownGoalConstPtr& goal);
	void actionCbSitDownObject(const hubero_ros_msgs::SitDownObjectGoalConstPtr& goal);
	void actionCbStand(const hubero_ros_msgs::StandGoalConstPtr& goal);
	void actionCbTalk(const hubero_ros_msgs::TalkGoalConstPtr& goal);
	void actionCbTalkObject(const hubero_ros_msgs::TalkObjectGoalConstPtr& goal);
	void actionCbTeleop(const hubero_ros_msgs::TeleopGoalConstPtr& goal);
	/// @}

	/**
	 * @brief Handler of the action goal callback
	 *
	 * @tparam Tresult type of the action result structure
	 * @tparam Tfeedback type of the action feedback structure
	 * @tparam Taction type of the action server
	 * @param request_processed_ok whether initial request status is ok
	 * @param task_type type of the task
	 * @param as_ptr action server pointer
	 */
	template <typename Tresult, typename Tfeedback, typename Taction>
	void actionCbHandler(bool request_processed_ok, TaskType task_type, ActionServerPtr<Taction>& as_ptr) {
		// evaluate request initial response
		if (!request_processed_ok || getTaskFeedbackType(task_type) == TASK_FEEDBACK_REJECTED) {
			Tresult result;
			result.result.text = "Rejected after initial check";
			result.result.status = actionlib_msgs::GoalStatus::REJECTED;
			as_ptr->setAborted(result);
			return;
		}

		// wait until accepted
		while (getTaskFeedbackType(task_type) == TASK_FEEDBACK_PENDING) {
			std::this_thread::sleep_for(TASK_FEEDBACK_PERIOD);
		}

		// activated
		TaskFeedbackType feedback_type = TASK_FEEDBACK_UNDEFINED;
		while ((feedback_type = getTaskFeedbackType(task_type)) == TASK_FEEDBACK_ACTIVE) {
			Tfeedback feedback;
			feedback.feedback.status = feedback_type;
			as_ptr->publishFeedback(feedback);
			std::this_thread::sleep_for(TASK_FEEDBACK_PERIOD);
		}

		auto final_feedback_type = getTaskFeedbackType(task_type);
		if (final_feedback_type == TASK_FEEDBACK_SUCCEEDED) {
			as_ptr->setSucceeded();
			return;
		} else if (final_feedback_type == TASK_FEEDBACK_ABORTED) {
			as_ptr->setAborted();
			return;
		}

		std::runtime_error(
			"failed to safely terminate requested task, finished with "
			+ std::to_string(final_feedback_type)
			+ " feedback type"
		);
	}
}; // class TaskRequestRos

} // namespace hubero
