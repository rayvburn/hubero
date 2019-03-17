/*
 * ActorROSInterface.cpp
 *
 *  Created on: Mar 17, 2019
 *      Author: rayvburn
 */

#include <ActorROSInterface.h>

namespace ActorUtils {

// ------------------------------------------------------------------- //

bool ActorROSInterface::is_node_started = false;
//ros::NodeHandle ActorROSInterface::nh;
std::unique_ptr<ros::NodeHandle> ActorROSInterface::nh;

// ------------------------------------------------------------------- //

ActorROSInterface::ActorROSInterface() {
	// empty constructor
}

// ------------------------------------------------------------------- //

void ActorROSInterface::Init(const std::string &_actor_name) {

	if ( !is_node_started ) {

		int argc = 0;
		char **argv = nullptr;
		ros::init(argc, argv, "actor_plugin_ros_interface_node");

		is_node_started = true;
		nh.reset(new ros::NodeHandle(("actor_plugin_ros_interface")));

	}

	pub_marker_array = nh->advertise<visualization_msgs::MarkerArray>("sfm_mrkr", 1000);
	this->actor_name = _actor_name;

}

// ------------------------------------------------------------------- //

void ActorROSInterface::PublishActorTf(const ignition::math::Pose3d &_actor_pose) {

	geometry_msgs::TransformStamped msg;

	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.child_frame_id = this->actor_name;

	msg.transform.translation.x = _actor_pose.Pos().X();
	msg.transform.translation.y = _actor_pose.Pos().Y();
	msg.transform.translation.z = 0.0f; // _actor_pose.Pos().Z();

	msg.transform.rotation.x = _actor_pose.Rot().X();
	msg.transform.rotation.y = _actor_pose.Rot().Y();
	msg.transform.rotation.z = _actor_pose.Rot().Z();
	msg.transform.rotation.w = _actor_pose.Rot().W();

	tf_broadcaster.sendTransform(msg);

}

// ------------------------------------------------------------------- //

void ActorROSInterface::PublishMarkerArray(const visualization_msgs::MarkerArray &_marker_array) {
	pub_marker_array.publish(_marker_array);
}

// ------------------------------------------------------------------- //

ActorROSInterface::~ActorROSInterface() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ActorUtils */
