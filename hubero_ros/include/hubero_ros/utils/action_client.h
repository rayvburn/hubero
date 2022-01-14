#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <hubero_common/defines.h>
#include <hubero_interfaces/task_request_base.h>
#include <hubero_ros/node.h>

#include <string>
#include <cinttypes>

namespace hubero {

/**
 * @brief Custom Action client aims to extend standard one with feedback information from action topic
 *
 * @details A specific action client aims to allow end-user to request a specific task from actor agent
 *
 * @tparam Taction type of the action
 * @tparam Tfeedback type of the feedback message
 */
template <typename Taction, typename Tfeedback>
class ActionClient: public actionlib::SimpleActionClient<Taction> {
public:
	/// Defines default size of the queue used for subscriber
	const int SUBSCRIBER_QUEUE_SIZE = 10;

	/**
	 * @brief Constructor of the Action Client
	 *
	 * @param node_ptr pointer to the @ref Node
	 * @param action_task_ns namespace of the action-related ROS topics; should contain action name after last slash
	 */
	ActionClient(
		std::shared_ptr<Node> node_ptr,
		const std::string& action_task_ns):
		actionlib::SimpleActionClient<Taction>(action_task_ns, true)
	{
		feedback_sub_ = node_ptr->getNodeHandlePtr()->subscribe(
			action_task_ns + "/" + "feedback",
			SUBSCRIBER_QUEUE_SIZE,
			&ActionClient::callbackFeedback, this
		);
		feedback_status_ = TaskFeedbackType::TASK_FEEDBACK_UNDEFINED;

		if (!ros::ok()) {
			ROS_ERROR("%s: Action server has not been started correctly (ROS is not running)", action_task_ns.c_str());
			return;
		}

		// Wait for server to become online
		while (!this->waitForServer(ros::Duration(5.0))) {
			ROS_INFO("%s: Waiting for the action server to come up: ", action_task_ns.c_str());
			// check if waitingForServer has been terminated by the process finish
			if (!ros::ok()) {
				ROS_ERROR("%s: Action server has not been started correctly", action_task_ns.c_str());
				return;
			}
		}

		ROS_INFO("%s: Action server started!", action_task_ns.c_str());
	}

	/**
	 * @defgroup feedback Feedback topics getters
	 *
	 * @note Types are hard-coded in the action definition file
	 * @{
	 */
	inline TaskFeedbackType getFeedbackStatus() const {
		return feedback_status_;
	}

	inline std::string getFeedbackText() const {
		return feedback_txt_;
	}
	/// @}

protected:
	ros::Subscriber feedback_sub_;
	std::string feedback_txt_;
	TaskFeedbackType feedback_status_;

	void callbackFeedback(const Tfeedback& msg) {
		feedback_txt_ = std::string(msg->feedback.text);
		feedback_status_ = static_cast<TaskFeedbackType>(msg->feedback.status);
	}
}; // class ActionClient
} // namespace hubero
