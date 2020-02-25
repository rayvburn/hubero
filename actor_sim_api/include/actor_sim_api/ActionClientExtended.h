/*
 * ActionClientExtended.h
 *
 *  Created on: Feb 25, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_SIM_API_ACTIONCLIENTEXTENDED_H_
#define INCLUDE_ACTOR_SIM_API_ACTIONCLIENTEXTENDED_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tuple>

template <typename Taction, typename Tfeedback>
class ActionClientExtended : public actionlib::SimpleActionClient<Taction> {

public:

	ActionClientExtended(const std::string &actor_ns_action_name, ros::NodeHandle *nh_ptr)
		: actionlib::SimpleActionClient<Taction>(actor_ns_action_name, true) {

		sub_feedback_ = nh_ptr->subscribe(actor_ns_action_name + "/feedback", 10, &ActionClientExtended::callbackFeedback, this);
		feedback_status_ = 0;

//		if ( !waitForServer(ros::Duration(10.0)) ) {
//			 // FIXME: error
//		}

	}

	int32_t getFeedbackStatus() { return (feedback_status_); }
	std::string getFeedbackText() {	return (feedback_txt_); }

	virtual ~ActionClientExtended() {}

private:

	ros::Subscriber sub_feedback_;
	std::string feedback_txt_;
	int32_t feedback_status_;

	void callbackFeedback(const Tfeedback& msg) {

		feedback_txt_ = std::string(msg->feedback.text);
		feedback_status_ = static_cast<int32_t>(msg->feedback.status);

	}

};

#endif /* INCLUDE_ACTOR_SIM_API_ACTIONCLIENTEXTENDED_H_ */
