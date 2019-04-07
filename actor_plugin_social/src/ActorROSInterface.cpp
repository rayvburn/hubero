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

	pub_array_grid = nh->advertise<visualization_msgs::MarkerArray>(_actor_name + "/sfm/grid", 10);
	pub_closest_points_array = nh->advertise<visualization_msgs::MarkerArray>(_actor_name + "/sfm/closest", 10);
	pub_marker_array_sf = nh->advertise<visualization_msgs::MarkerArray>(_actor_name + "/sfm/sf_marker", 10);
	pub_marker_array = nh->advertise<visualization_msgs::MarkerArray>(_actor_name + "/sfm/marker_array", 10);// FIXME: these are closest in fact
	pub_marker = nh->advertise<visualization_msgs::Marker>(_actor_name + "/sfm/marker_single", 10);
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

void ActorROSInterface::PublishTargetTf(const ignition::math::Vector3d &_target_pos) {

	geometry_msgs::TransformStamped msg;

	std::string frame = this->actor_name;
	frame.append("_goal");

	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.child_frame_id = frame;

	msg.transform.translation.x = _target_pos.X();
	msg.transform.translation.y = _target_pos.Y();
	msg.transform.translation.z = 0.0f; // _actor_pose.Pos().Z();

	msg.transform.rotation.x = 0.0;
	msg.transform.rotation.y = 0.0;
	msg.transform.rotation.z = 0.0;
	msg.transform.rotation.w = 1.0;

	tf_broadcaster.sendTransform(msg);

}

// ------------------------------------------------------------------- //

void ActorROSInterface::PublishMarker(const visualization_msgs::Marker &_marker) {

	/*
	 * switch (vis_type) {
	 * case(1):
	 * 	if ( getNumSub > 0 ) {
	 * 		publish(_marker);
	 * 	}
	 * 	break;
	 * }
	 *
	 */
	pub_marker.publish(_marker);
}

// ------------------------------------------------------------------- //

void ActorROSInterface::PublishMarkerArray(const visualization_msgs::MarkerArray &_marker_array) {
	pub_marker_array.publish(_marker_array);
}

// ------------------------------------------------------------------- //

void ActorROSInterface::PublishMarkerArrayGrid(const visualization_msgs::MarkerArray &_marker_array) {
	if ( pub_array_grid.getNumSubscribers() > 0 ) {
		pub_array_grid.publish(_marker_array);
	}
}
// ------------------------------------------------------------------- //

void ActorROSInterface::PublishClosestPoints(const visualization_msgs::MarkerArray &_marker_array) {
	pub_closest_points_array.publish(_marker_array);
}

// ------------------------------------------------------------------- //

void ActorROSInterface::PublishSFArrows(const visualization_msgs::MarkerArray &_marker_array) {
	pub_marker_array_sf.publish(_marker_array);
}

// ------------------------------------------------------------------- //

unsigned int ActorROSInterface::getGridSubscribersNum() const {
	return ( pub_array_grid.getNumSubscribers() );
}

// ------------------------------------------------------------------- //

ActorROSInterface::~ActorROSInterface() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ActorUtils */
