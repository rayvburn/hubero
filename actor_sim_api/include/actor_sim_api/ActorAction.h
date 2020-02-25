/*
 * ActorAction.h
 *
 *  Created on: Feb 24, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_SIM_API_ACTORACTION_H_
#define INCLUDE_ACTOR_SIM_API_ACTORACTION_H_


#include <actor_sim_action/FollowObjectAction.h>
#include <actor_sim_action/LieDownAction.h>
#include <actor_sim_action/LieDownNameAction.h>
#include <actor_sim_action/SetGoalAction.h>
#include <actor_sim_action/SetGoalNameAction.h>
#include "ActorAPI.h"
#include "ActionClientExtended.h"

class ActorAction : public ActorAPI {

public:

	/// \brief Parametrized constructor
	ActorAction(const std::string &actor_ns_name, ros::NodeHandle *nh_ptr);

	void followObject(const std::string &name);
	void lieDown(const double &x, const double &y, const double &z, const double &rotation, const std::string &frame = "world");
	void lieDownName(const std::string &name_object, const double &lying_height, const double &rotation);
	void setGoal(const double &x, const double &y, const std::string &frame = "world");
	void setGoalName(const std::string &name_object);

	// tuple components' types are hard-coded in the action files
	std::tuple<int32_t, std::string> getFeedbackFollowObject();
	std::tuple<int32_t, std::string> getFeedbackLieDown();
	std::tuple<int32_t, std::string> getFeedbackLieDownName();
	std::tuple<int32_t, std::string> getFeedbackSetGoal();
	std::tuple<int32_t, std::string> getFeedbackSetGoalName();

	// it would be the best if feedback can be achieved via a single
	// method for every simple action client;
	// what's a problem here is storing a general pointer to the SimpleActionClient,
	// which is a template class
	// std::tuple<int32_t, std::string> getActionFeedback();

	virtual ~ActorAction();

protected:

	// pointers used here as SAC it may create an internal ROS NodeHandle - pointer creation may be necessary
	ActionClientExtended<actor_sim_action::FollowObjectAction, actor_sim_action::FollowObjectActionFeedbackConstPtr>* 	ac_follow_object_ptr_;
	ActionClientExtended<actor_sim_action::LieDownAction, actor_sim_action::LieDownActionFeedbackConstPtr>* 			ac_lie_down_ptr_;
	ActionClientExtended<actor_sim_action::LieDownNameAction, actor_sim_action::LieDownNameActionFeedbackConstPtr>* 	ac_lie_down_name_ptr_;
	ActionClientExtended<actor_sim_action::SetGoalAction, actor_sim_action::SetGoalActionFeedbackConstPtr>* 			ac_set_goal_ptr_;
	ActionClientExtended<actor_sim_action::SetGoalNameAction, actor_sim_action::SetGoalNameActionFeedbackConstPtr>* 	ac_set_goal_name_ptr_;

};

#endif /* INCLUDE_ACTOR_SIM_API_ACTORACTION_H_ */
