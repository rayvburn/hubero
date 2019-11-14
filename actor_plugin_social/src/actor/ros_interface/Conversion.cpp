/*
 * Conversion.cpp
 *
 *  Created on: Jun 22, 2019
 *      Author: rayvburn
 */

#include <actor/ros_interface/Conversion.h>

namespace actor {
namespace ros_interface {

// ------------------------------------------------------------------- //
// helper functions -------------------------------------------------- //
// ------------------------------------------------------------------- //
geometry_msgs::PoseStamped Conversion::convertIgnVectorToPoseStamped (const ignition::math::Vector3d &pos, bool zero_height) {

	geometry_msgs::PoseStamped pose_stamped;

	pose_stamped.header.frame_id = "map"; // world?
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.pose.position.x = pos.X();
	pose_stamped.pose.position.y = pos.Y();
	if ( !zero_height ) {
		pose_stamped.pose.position.z = pos.Z();
	} else {
		pose_stamped.pose.position.z = 0.0;
	}
	pose_stamped.pose.orientation.x = 0.0f;
	pose_stamped.pose.orientation.y = 0.0f;
	pose_stamped.pose.orientation.z = 0.0f;
	pose_stamped.pose.orientation.w = 1.0f;

	return (pose_stamped);

}

// ------------------------------------------------------------------- //

geometry_msgs::PoseStamped Conversion::convertIgnPoseToPoseStamped (const ignition::math::Pose3d &pose, bool zero_height) {

	geometry_msgs::PoseStamped pose_stamped;

	pose_stamped.header.frame_id = "map"; // world?
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.pose.position.x = pose.Pos().X();
	pose_stamped.pose.position.y = pose.Pos().Y();
	if ( !zero_height ) {
		pose_stamped.pose.position.z = pose.Pos().Z();
	} else {
		pose_stamped.pose.position.z = 0.0;
	}
	pose_stamped.pose.orientation.x = pose.Rot().X();
	pose_stamped.pose.orientation.y = pose.Rot().Y();
	pose_stamped.pose.orientation.z = pose.Rot().Z();
	pose_stamped.pose.orientation.w = pose.Rot().W();

	return (pose_stamped);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Conversion::convertPoseStampedToIgnVector (const geometry_msgs::PoseStamped &pose) {

	ignition::math::Vector3d pos;

	pos.X(pose.pose.position.x);
	pos.Y(pose.pose.position.y);
	pos.Z(pose.pose.position.z);

	return (pos);

}

// ------------------------------------------------------------------- //

ignition::math::Pose3d Conversion::convertPoseStampedToIgnPose (const geometry_msgs::PoseStamped &pose) {

	ignition::math::Pose3d pose_ign;

	pose_ign.Pos().X(pose.pose.position.x);
	pose_ign.Pos().Y(pose.pose.position.y);
	pose_ign.Pos().Z(pose.pose.position.z);

	pose_ign.Rot().X(pose.pose.orientation.x);
	pose_ign.Rot().Y(pose.pose.orientation.y);
	pose_ign.Rot().Z(pose.pose.orientation.z);
	pose_ign.Rot().W(pose.pose.orientation.w);

	return (pose_ign);

}

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
