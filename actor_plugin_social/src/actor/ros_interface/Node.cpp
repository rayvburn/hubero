/*
 * Node.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#include <actor/ros_interface/Node.h>
#include <ros/ros.h>

namespace actor {
namespace ros_interface {

// ------------------------------------------------------------------- //

bool actor::ros_interface::Node::node_started_ = false;
std::shared_ptr<ros::NodeHandle> actor::ros_interface::Node::nh_ptr_;

// ------------------------------------------------------------------- //

Node::Node() {

	if ( !node_started_ ) {

		// initialize ROS' node
		int argc = 0;
		char **argv = nullptr;
		ros::init(argc, argv, "actor_plugin_ros_interface_node");

		// flag to create only 1 node for all actors
		node_started_ = true;

		// create new NodeHandle in a private namespace
		nh_ptr_.reset(new ros::NodeHandle(("~actor_plugin_ros_interface")));

	}

}

// ------------------------------------------------------------------- //

std::shared_ptr<ros::NodeHandle> Node::getNodeHandlePtr() const {
	return (nh_ptr_);
}

// ------------------------------------------------------------------- //

Node::~Node() { }

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */