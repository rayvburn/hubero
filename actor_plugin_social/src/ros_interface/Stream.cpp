/*
 * Stream.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#include "ros_interface/Stream.h"

namespace actor {
namespace ros_interface {

// ------------------------------------------------------------------- //

Stream::Stream() { }

// ------------------------------------------------------------------- //

void Stream::setNodeHandle(std::shared_ptr<::ros::NodeHandle> nh_ptr) {
	nh_ptr_ = nh_ptr;
}

// ------------------------------------------------------------------- //

void Stream::setNamespace(const std::string &ns) {
	namespace_ = ns;
}

// ------------------------------------------------------------------- //

std::tuple<bool, ros::Publisher> Stream::findPublisherInMap(const unsigned int &id) {

	std::map<unsigned int, ros::Publisher>::const_iterator it;
	it = publisher_id_map_.find(id);
	if ( it != publisher_id_map_.end() ) {
		return (std::make_tuple( true, it->second ));
	}
	return (std::make_tuple( false, ros::Publisher() ));

}

// ------------------------------------------------------------------- //

geometry_msgs::TransformStamped Stream::convertPoseToTfStamped(const ignition::math::Pose3d &pose) const {

	geometry_msgs::TransformStamped tf_stamp;
	tf_stamp.transform.translation.x = pose.Pos().X();
	tf_stamp.transform.translation.y = pose.Pos().Y();
	tf_stamp.transform.translation.z = 0.0f; // _actor_pose.Pos().Z();

	tf_stamp.transform.rotation.x = pose.Rot().X();
	tf_stamp.transform.rotation.y = pose.Rot().Y();
	tf_stamp.transform.rotation.z = pose.Rot().Z();
	tf_stamp.transform.rotation.w = pose.Rot().W();

	return (tf_stamp);

}

// ------------------------------------------------------------------- //

Stream::~Stream() { }

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
