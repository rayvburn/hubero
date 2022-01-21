#include <hubero_ros/node.h>
#include <ros/ros.h>

#include <iostream>

namespace hubero {

bool Node::node_started_ = false;
std::string Node::namespace_;
std::string Node::simulator_frame_id_;
std::string Node::task_namespace_;
std::shared_ptr<ros::NodeHandle> Node::nh_ptr_;

Node::Node(const std::string& node_name) {
	if (Node::node_started_) {
		return;
	}

	// initialize ROS node
	int argc = 0;
	char **argv = nullptr;
	ros::init(argc, argv, node_name);

	// find namespace value
	ros::NodeHandle nh;
	std::string namespace_ros_param;
	nh.searchParam("/hubero_ros/namespace", namespace_ros_param);
	nh.param(namespace_ros_param, Node::namespace_, std::string("hubero_ros"));

	// find simulator_frame value
	std::string sim_frame_ros_param;
	nh.searchParam("/hubero_ros/simulator_frame", sim_frame_ros_param);
	nh.param(sim_frame_ros_param, Node::simulator_frame_id_, std::string("world"));

	// find task namespace value
	std::string task_namespace_ros_param;
	nh.searchParam("/hubero_ros/task_namespace", task_namespace_ros_param);
	nh.param(task_namespace_ros_param, Node::task_namespace_, std::string("task"));

	// create new NodeHandle in a global namespace
	nh_ptr_.reset(new ros::NodeHandle(("/" + Node::namespace_)));

	// flag to create only 1 node for all actors
	Node::node_started_ = true;
}

} // namespace hubero
