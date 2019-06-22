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
#include <tf/transform_listener.h>

namespace actor {
namespace ros_interface {

class Node {

public:

	/// \brief Default constructor; initializes ROS node
	Node();

	/// \brief Copy a shared_ptr instance for use by other class
	std::shared_ptr<ros::NodeHandle> getNodeHandlePtr() const;

	/// \brief Copy a shared_ptr instance for use by other class
	std::shared_ptr<tf::TransformListener> getTfListenerPtr() const;

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

	/// \brief Transform listener's shared_ptr which is used by
	/// global planner; it's safe to initialize it just after ros::init()
	/// and NodeHandle, otherwise there may be some application hang
	/// just after start; all classes use the same tf_listener
	static std::shared_ptr<tf::TransformListener> tf_listener_ptr_;

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_NODE_H_ */
