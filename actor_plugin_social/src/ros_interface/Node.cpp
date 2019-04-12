/*
 * Node.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#include "ros_interface/Node.h"

namespace actor {
namespace ros_interface {

// ------------------------------------------------------------------- //

bool actor::ros_interface::Node::node_started_ = false;
std::shared_ptr<ros::NodeHandle> actor::ros_interface::Node::nh_ptr_;

// ------------------------------------------------------------------- //

Node::Node() {

	if ( !node_started_ ) {

		std::cout << "NODE STARTED SUCCESSFULLY" << std::endl;
		int argc = 0;
		char **argv = nullptr;
		ros::init(argc, argv, "actor_plugin_ros_interface_node");

		node_started_ = true;
		nh_ptr_.reset(new ros::NodeHandle(("actor_plugin_ros_interface")));

	}

}

// ------------------------------------------------------------------- //

std::shared_ptr<::ros::NodeHandle> Node::getNodeHandlePtr() const {

	// TODO: check if for sure 1 NH is created
	return (nh_ptr_);

}

// ------------------------------------------------------------------- //

Node::~Node() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
