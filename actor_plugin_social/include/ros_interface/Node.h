/*
 * Node.h
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ROS_INTERFACE_NODE_H_
#define INCLUDE_ROS_INTERFACE_NODE_H_

#include <memory> 		// std::unique_ptr
#include <ros/ros.h> 	// ros::NodeHandle

namespace actor {
namespace ros_interface {

class Node {

public:

	Node();
	std::shared_ptr<ros::NodeHandle> getNodeHandlePtr() const;
	virtual ~Node();

private:

	static bool node_started_;
	static std::shared_ptr<::ros::NodeHandle> nh_ptr_;

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_NODE_H_ */
