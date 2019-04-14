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

void Stream::initPublisher(const ActorMarkerType &type, const std::string &topic_name) {

	ros::Publisher publisher;
	publisher = nh_ptr_->advertise<visualization_msgs::Marker>(namespace_ + topic_name, 10);
	publisher_id_map_.insert( std::pair<unsigned int, ros::Publisher> (static_cast<unsigned int>(type), publisher) );

}

// ------------------------------------------------------------------- //

void Stream::initPublisher(const ActorMarkerArrayType &type, const std::string &topic_name) {

	ros::Publisher publisher;
	publisher = nh_ptr_->advertise<visualization_msgs::MarkerArray>(namespace_ + topic_name, 10);
	publisher_id_map_.insert( std::pair<unsigned int, ros::Publisher> (static_cast<unsigned int>(type), publisher) );

}

// ------------------------------------------------------------------- //

/*
template <typename T1, typename T2>
void Stream::publishData(const T1 type, const T2 marker_msg) {

	ros::Publisher pub;
	bool found = false;
	std::tie(found, pub) = findPublisherInMap( static_cast<unsigned int>(type) );

	if ( found ) {
		pub.publish(marker_msg);
	}

}
*/

// ------------------------------------------------------------------- //

void Stream::publishData(const ActorMarkerType &type, const visualization_msgs::Marker &marker) {

	ros::Publisher pub;
	bool found = false;
	std::tie(found, pub) = findPublisherInMap( static_cast<unsigned int>(type) );

	if ( found && pub.getNumSubscribers() > 0 ) {
		pub.publish(marker);
	}

}

// ------------------------------------------------------------------- //

void Stream::publishData(const ActorMarkerArrayType &type, const visualization_msgs::MarkerArray &marker) {

	ros::Publisher pub;
	bool found = false;
	std::tie(found, pub) = findPublisherInMap( static_cast<unsigned int>(type) );

	if ( found && pub.getNumSubscribers() > 0 ) {
		pub.publish(marker);
	}

}

// ------------------------------------------------------------------- //

void Stream::publishData(const ActorTfType &type, const ignition::math::Pose3d &pose) {

	// append _goal to a namespace when broadcasting target pose
	std::string child_frame = namespace_;
	if ( type == ACTOR_TF_TARGET ) {
		child_frame.append("_goal");
	}

	geometry_msgs::TransformStamped tf_stamp = convertPoseToTfStamped(pose);
	tf_stamp.header.frame_id = "map";
	tf_stamp.header.stamp = ros::Time::now();
	tf_stamp.child_frame_id = child_frame;

	tf_broadcaster_.sendTransform(tf_stamp);

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
