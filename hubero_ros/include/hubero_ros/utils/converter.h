#pragma once

#include <hubero_common/typedefs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

namespace hubero {

geometry_msgs::Pose ignPoseToMsgPose(const Pose3& pose);
geometry_msgs::Transform ignPoseToMsgTf(const Pose3& pose);
Vector3 msgTwistToIgnVector(const geometry_msgs::Twist& twist);

} // namespace hubero
