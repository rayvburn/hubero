#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/typedefs.h>

#include <hubero_ros/utils/action_client.h>
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

#include <memory>

namespace hubero {

/**
 * @brief Class that stands for a client for @ref TaskRequestRos class
 *
 * @details This class can be directly used to request tasks via ROS Action Servers as long as @ref TaskRequestRos
 * interface for a specific actor is operational. @ref TaskRequestRosApi wraps actionlib client and is intended
 * to be allocated in C++ scenario executables with HuBeRo actors.
 */
class TaskRequestRosApi {
public:
	/**
	 * @brief Constructor of @ref TaskRequestRosApi instance
	 * @param actor_name actor identifier from simulation
	 */
	TaskRequestRosApi(const std::string& actor_name);

	/**
	 * @defgroup tasks Methods that allow to request specific task from controlled actor
	 * @{
	 */
	/**
	 * @brief Requests actor to follow object that in simulation is recognized by @ref object_name
	 */
	bool followObject(const std::string& object_name);

	/**
	 * @brief Requests actor to follow object according to @ref goal
	 */
	bool followObject(const hubero_ros_msgs::FollowObjectGoal& goal);

	/**
	 * @brief Requests actor to lie down at @ref pos point with orientation @ref yaw
	 * @details @ref pos frame (coordinate system) of reference is given by @ref frame_id
	 */
	bool lieDown(const Vector3& pos, const double& yaw, const std::string& frame_id);

	/**
	 * @brief Requests actor to lie down according to @ref goal
	 */
	bool lieDown(const hubero_ros_msgs::LieDownGoal& goal);

	/**
	 * @brief Requests actor to lie down onto @ref object_name object with orientation @ref yaw at @ref height height
	 */
	bool lieDownObject(const std::string& object_name, const double& height, const double& yaw);

	/**
	 * @brief Requests actor to lie down onto object according to @ref goal
	 */
	bool lieDownObject(const hubero_ros_msgs::LieDownObjectGoal& goal);

	/**
	 * @brief Requests actor to move around (choose his navigation goals randomly)
	 */
	bool moveAround();

	/**
	 * @brief Requests actor to move around (choose his navigation goals randomly)
	 */
	bool moveAround(const hubero_ros_msgs::MoveAroundGoal& goal);

	/**
	 * @brief Requests actor to move to @ref pos point, where @ref pos frame of reference is given by @ref frame_id
	 */
	bool moveToGoal(const Vector3& pos, const std::string& frame_id);

	/**
	 * @brief Requests actor to move to goal according to @ref goal
	 */
	bool moveToGoal(const hubero_ros_msgs::MoveToGoalGoal& goal);

	/**
	 * @brief Requests actor to move to specific object named @ref object_name
	 */
	bool moveToObject(const std::string& object_name);

	/**
	 * @brief Requests actor to move to specific object according to @ref goal
	 */
	bool moveToObject(const hubero_ros_msgs::MoveToObjectGoal& goal);

	/**
	 * @brief Requests actor to run to @ref pos point, where @ref pos frame of reference is given by @ref frame_id
	 */
	bool run(const Vector3& pos, const std::string& frame_id);

	/**
	 * @brief Requests actor to run to goal point according to @ref goal
	 */
	bool run(const hubero_ros_msgs::RunGoal& goal);

	/**
	 * @brief Requests actor to sit down at @ref pos point with orientation @ref yaw
	 * @details @ref pos frame (coordinate system) of reference is given by @ref frame_id
	 */
	bool sitDown(const Vector3& pos, const double& yaw, const std::string& frame_id);

	/**
	 * @brief Requests actor to sit down according to @ref goal
	 */
	bool sitDown(const hubero_ros_msgs::SitDownGoal& goal);

	/**
	 * @brief Requests actor to sit down onto @ref object_name object with orientation @ref yaw at @ref height height
	 */
	bool sitDownObject(const std::string& object_name, const double& height, const double& yaw);

	/**
	 * @brief Requests actor to sit down onto object according to @ref goal
	 */
	bool sitDownObject(const hubero_ros_msgs::SitDownObjectGoal& goal);

	/**
	 * @brief Requests actor to stand still
	 */
	bool stand();

	/**
	 * @brief Requests actor to stand still according to @ref goal
	 */
	bool stand(const hubero_ros_msgs::StandGoal& goal);

	/**
	 * @brief Requests actor to talk, but first let it move to @ref pos position expressed in @ref frame_id frame
	 */
	bool talk(const Vector3& pos, const std::string& frame_id);

	/**
	 * @brief Requests actor to talk according to @ref goal
	 */
	bool talk(const hubero_ros_msgs::TalkGoal& goal);

	/**
	 * @brief Requests actor to talk to object (move towards that object and start talking) given by @ref object_name
	 */
	bool talkObject(const std::string& object_name);

	/**
	 * @brief Requests actor to talk to object according to @ref goal
	 * @details Also @see talkObject
	 */
	bool talkObject(const hubero_ros_msgs::TalkObjectGoal& goal);

	/**
	 * @brief Requests actor to teleoperate
	 */
	bool teleop();

	/**
	 * @brief Requests actor to teleoperate according to @ref goal
	 */
	bool teleop(const hubero_ros_msgs::TeleopGoal& goal);
	/// @}

protected:
	std::shared_ptr<Node> node_ptr_;

	/// Alias declaration
	template <typename Taction, typename Tfeedback>
	using ActionClientPtr = std::shared_ptr<ActionClient<Taction, Tfeedback>>;

	ActionClientPtr<hubero_ros_msgs::FollowObjectAction, hubero_ros_msgs::FollowObjectActionFeedbackConstPtr> ac_follow_object_ptr_;
	ActionClientPtr<hubero_ros_msgs::LieDownAction, hubero_ros_msgs::LieDownActionFeedbackConstPtr> ac_lie_down_ptr_;
	ActionClientPtr<hubero_ros_msgs::LieDownObjectAction, hubero_ros_msgs::LieDownObjectActionFeedbackConstPtr> ac_lie_down_object_ptr_;
	ActionClientPtr<hubero_ros_msgs::MoveAroundAction, hubero_ros_msgs::MoveAroundActionFeedbackConstPtr> ac_move_around_ptr_;
	ActionClientPtr<hubero_ros_msgs::MoveToGoalAction, hubero_ros_msgs::MoveToGoalActionFeedbackConstPtr> ac_move_to_goal_ptr_;
	ActionClientPtr<hubero_ros_msgs::MoveToObjectAction, hubero_ros_msgs::MoveToObjectActionFeedbackConstPtr> ac_move_to_object_ptr_;
	ActionClientPtr<hubero_ros_msgs::RunAction, hubero_ros_msgs::RunActionFeedbackConstPtr> ac_run_ptr_;
	ActionClientPtr<hubero_ros_msgs::SitDownAction, hubero_ros_msgs::SitDownActionFeedbackConstPtr> ac_sit_down_ptr_;
	ActionClientPtr<hubero_ros_msgs::SitDownObjectAction, hubero_ros_msgs::SitDownObjectActionFeedbackConstPtr> ac_sit_down_object_ptr_;
	ActionClientPtr<hubero_ros_msgs::StandAction, hubero_ros_msgs::StandActionFeedbackConstPtr> ac_stand_ptr_;
	ActionClientPtr<hubero_ros_msgs::TalkAction, hubero_ros_msgs::TalkActionFeedbackConstPtr> ac_talk_ptr_;
	ActionClientPtr<hubero_ros_msgs::TalkObjectAction, hubero_ros_msgs::TalkObjectActionFeedbackConstPtr> ac_talk_object_ptr_;
	ActionClientPtr<hubero_ros_msgs::TeleopAction, hubero_ros_msgs::TeleopActionFeedbackConstPtr> ac_teleop_ptr_;
}; // class TaskRequestRosApi
} // namespace hubero
