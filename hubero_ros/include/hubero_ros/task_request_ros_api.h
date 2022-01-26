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
	TaskRequestRosApi(const std::string& actor_name);

	bool followObject(const std::string& object_name);

	bool followObject(const hubero_ros_msgs::FollowObjectGoal& goal);

	bool lieDown(const Vector3& pos, const double& yaw, const std::string& frame_id);

	bool lieDown(const hubero_ros_msgs::LieDownGoal& goal);

	bool lieDownObject(const std::string& object_name, const double& height, const double& yaw);

	bool lieDownObject(const hubero_ros_msgs::LieDownObjectGoal& goal);

	bool moveAround();

	bool moveAround(const hubero_ros_msgs::MoveAroundGoal& goal);

	bool moveToGoal(const Vector3& pos, const std::string& frame_id);

	bool moveToGoal(const hubero_ros_msgs::MoveToGoalGoal& goal);

	bool moveToObject(const std::string& object_name);

	bool moveToObject(const hubero_ros_msgs::MoveToObjectGoal& goal);

	bool run(const Vector3& pos, const std::string& frame_id);

	bool run(const hubero_ros_msgs::RunGoal& goal);

	bool sitDown(const Vector3& pos, const double& yaw, const std::string& frame_id);

	bool sitDown(const hubero_ros_msgs::SitDownGoal& goal);

	bool sitDownObject(const std::string& object_name, const double& height, const double& yaw);

	bool sitDownObject(const hubero_ros_msgs::SitDownObjectGoal& goal);

	bool stand();

	bool stand(const hubero_ros_msgs::StandGoal& goal);

	bool talk(const Vector3& pos, const std::string& frame_id);

	bool talk(const hubero_ros_msgs::TalkGoal& goal);

	bool talkObject(const std::string& object_name);

	bool talkObject(const hubero_ros_msgs::TalkObjectGoal& goal);

	bool teleop();

	bool teleop(const hubero_ros_msgs::TeleopGoal& goal);

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
