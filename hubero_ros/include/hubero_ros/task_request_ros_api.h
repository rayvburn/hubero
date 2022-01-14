#pragma once

#include <hubero_common/defines.h>

#include <hubero_ros/utils/action_client.h>
#include <hubero_ros/node.h>

#include <hubero_ros_msgs/FollowObjectAction.h>
#include <hubero_ros_msgs/LieDownAction.h>
#include <hubero_ros_msgs/LieDownObjectAction.h>
#include <hubero_ros_msgs/MoveToGoalAction.h>
#include <hubero_ros_msgs/MoveToObjectAction.h>

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

	// FIXME: use ROS msgs
	bool lieDown();

	bool lieDownObject(const std::string& object_name);

	// FIXME: use ROS msgs
	bool moveToGoal();

	bool moveToObject(const std::string& object_name);

protected:
	std::shared_ptr<Node> node_ptr_;

	/// Alias declaration
	template <typename Taction, typename Tfeedback>
	using ActionClientPtr = std::shared_ptr<ActionClient<Taction, Tfeedback>>;

	ActionClientPtr<hubero_ros_msgs::FollowObjectAction, hubero_ros_msgs::FollowObjectActionFeedbackConstPtr> ac_follow_object_ptr_;
	ActionClientPtr<hubero_ros_msgs::LieDownAction, hubero_ros_msgs::LieDownActionFeedbackConstPtr> ac_lie_down_ptr_;
	ActionClientPtr<hubero_ros_msgs::LieDownObjectAction, hubero_ros_msgs::LieDownObjectActionFeedbackConstPtr> ac_lie_down_object_ptr_;
	ActionClientPtr<hubero_ros_msgs::MoveToGoalAction, hubero_ros_msgs::MoveToGoalActionFeedbackConstPtr> ac_move_to_goal_ptr_;
	ActionClientPtr<hubero_ros_msgs::MoveToObjectAction, hubero_ros_msgs::MoveToObjectActionFeedbackConstPtr> ac_move_to_object_ptr_;

}; // class TaskRequestRosApi
} // namespace hubero
