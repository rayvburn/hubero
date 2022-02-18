#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/typedefs.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

#include <actionlib_msgs/GoalStatus.h>
#include <actionlib/client/simple_client_goal_state.h>

namespace hubero {

geometry_msgs::Point ignVectorToMsgPoint(const Vector3& position);
geometry_msgs::Pose ignPoseToMsgPose(const Pose3& pose);
geometry_msgs::Transform ignPoseToMsgTf(const Pose3& pose);
geometry_msgs::Twist ignVectorsToMsgTwist(const Vector3& vel_lin, const Vector3& vel_ang);
geometry_msgs::Quaternion ignVectorRpyToMsgQuaternion(const Vector3& rpy);
Vector3 msgTwistToIgnVector(const geometry_msgs::Twist& twist);
Pose3 msgTfToPose(const geometry_msgs::Transform& tf);
Vector3 msgPointToIgnVector(const geometry_msgs::Point& point);
Pose3 msgPoseToIgnPose(const geometry_msgs::Pose& pose);

/**
 * @brief Transforms actionlib_msgs::GoalStatus enum to hubero::TaskFeedbackType enum
 */
TaskFeedbackType convertActionStatusToTaskFeedback(const uint8_t& status);

/**
 * @brief Transforms actionlib::SimpleClientGoalState::StateEnum enum to hubero::TaskFeedbackType enum
 */
TaskFeedbackType convertSimpleClientStateToTaskFeedback(const actionlib::SimpleClientGoalState::StateEnum& status);

} // namespace hubero
