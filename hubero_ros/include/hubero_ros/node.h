#pragma once

#include <ros/ros.h>
#include <memory>

namespace hubero {

class Node {
public:
	/**
	 * @brief Default constructor; initializes ROS node
	 */
	Node();

	/**
	 * @brief Retrieves namespace name that was read via ROS param
	 */
	static inline std::string getNamespaceName() {
		return Node::namespace_;
	}

	/**
	 * @brief Retrieves simulator frame name that was read via ROS param
	 */
	static inline std::string getSimulatorFrame() {
		return Node::simulator_frame_id_;
	}

	/**
	 * @brief Copy a shared_ptr instance for use by other class
	 */
	static inline std::shared_ptr<ros::NodeHandle> getNodeHandlePtr() {
		return Node::nh_ptr_;
	}

protected:
	/**
	 * @brief Flag to indicate whether ROS node was initialized
	 * @note Prevents creating separate node for each Actor class object
	 */
	static bool node_started_;

	/**
	 * @brief Defines shared namespace of the HuBeRo topics/actions etc.
	 */
	static std::string namespace_;

	/**
	 * @brief Defines name of the frame that poses from simulator are using
	 */
	static std::string simulator_frame_id_;

	/**
	 * @brief NodeHandle's shared_ptr which is passed to other classes to provide interface with ROS.
	 * @details All of them use the same instance of NodeHandle
	 */
	static std::shared_ptr<ros::NodeHandle> nh_ptr_;
}; // class Node

} // namespace hubero
