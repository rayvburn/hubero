/*
 * Conversion.h
 *
 *  Created on: Jun 22, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_ROS_INTERFACE_CONVERSION_H_
#define INCLUDE_ACTOR_ROS_INTERFACE_CONVERSION_H_

#include <geometry_msgs/PoseStamped.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <string>

namespace actor {
namespace ros_interface {

class Conversion {

public:

// helper functions --------------------------------------------------------------------------
static geometry_msgs::PoseStamped 	convertIgnVectorToPoseStamped	(const ignition::math::Vector3d 	&pos, 	const std::string &frame = "world",	bool zero_height = false);
static geometry_msgs::PoseStamped 	convertIgnPoseToPoseStamped		(const ignition::math::Pose3d 		&pose, 	const std::string &frame = "world",	bool zero_height = false);
static ignition::math::Vector3d 	convertPoseStampedToIgnVector	(const geometry_msgs::PoseStamped 	&pose);
static ignition::math::Pose3d 		convertPoseStampedToIgnPose		(const geometry_msgs::PoseStamped 	&pose);
// -------------------------------------------------------------------------------------------

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_ROS_INTERFACE_CONVERSION_H_ */
