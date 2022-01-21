#pragma once

#include <hubero_common/typedefs.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>

namespace hubero {

geometry_msgs::Pose ignPoseToMsgPose(const Pose3& pose);
geometry_msgs::Transform ignPoseToMsgTf(const Pose3& pose);
geometry_msgs::Twist ignVectorsToMsgTwist(const Vector3& vel_lin, const Vector3& vel_ang);
Vector3 msgTwistToIgnVector(const geometry_msgs::Twist& twist);
Pose3 msgTfToPose(const geometry_msgs::Transform& tf);

} // namespace hubero
