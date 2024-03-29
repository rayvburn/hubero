#pragma once

#include <ros/ros.h>
#include <memory>

namespace hubero {

/**
 * @brief ROS NodeHandle with main parameters of HuBeRo ROS interface
 */
class Node {
public:
	/**
	 * @brief Constructor that initializes ROS node named @ref node_name
	 */
	Node(const std::string& node_name = std::string("hubero_ros_node"));

	/**
	 * @brief Retrieves main namespace name that was read via ROS param
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
	 * @brief Retrieves task namespace name that was read via ROS param
	 */
	static inline std::string getTaskNamespaceName() {
		return Node::task_namespace_;
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

	/// @brief Defines shared namespace of the HuBeRo topics/actions etc.
	static std::string namespace_;

	/// @brief Defines name of the frame that poses from simulator are using
	static std::string simulator_frame_id_;

	/// @brief Defines shared namespace of the HuBeRo topics that allow task requesting
	static std::string task_namespace_;

	/**
	 * @brief NodeHandle's shared_ptr which is passed to other classes to provide interface with ROS.
	 * @details All of them use the same instance of NodeHandle
	 */
	static std::shared_ptr<ros::NodeHandle> nh_ptr_;
}; // class Node

} // namespace hubero
