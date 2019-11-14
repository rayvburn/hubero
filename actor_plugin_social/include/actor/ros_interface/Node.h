/*
 * Node.h
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_ROS_INTERFACE_NODE_H_
#define INCLUDE_ACTOR_ROS_INTERFACE_NODE_H_

#include <memory> 		// std::shared_ptr
#include <ros/ros.h> 	// ros::NodeHandle

namespace actor {
namespace ros_interface {

class Node {

public:

	/// \brief Default constructor; initializes ROS node
	Node();

	/// \brief Copy a shared_ptr instance for use by other class
	std::shared_ptr<ros::NodeHandle> getNodeHandlePtr() const;

	/// \brief Default destructor
	virtual ~Node();

private:

	/// \brief Flag to indicate whether ROS node was initialized;
	/// prevents creating separate node for each Actor class object
	static bool node_started_;

	/// \brief NodeHandle's shared_ptr which is passed
	/// to other classes which provide interface with ROS;
	/// all of them use the same instance of NodeHandle
	static std::shared_ptr<ros::NodeHandle> nh_ptr_;

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_ROS_INTERFACE_NODE_H_ */
