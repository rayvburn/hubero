/*
 * ActorAction.cpp
 *
 *  Created on: Feb 24, 2020
 *      Author: rayvburn
 */

#include <actor_sim_api/ActorAction.h>

// ---------------------------------------------------------------------------------

ActorAction::ActorAction(const std::string &actor_ns_name, ros::NodeHandle *nh_ptr)
	: ActorAPI(actor_ns_name, nh_ptr)
{

	// note: Constructor with namespacing options.
	// https://docs.ros.org/kinetic/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#a47b97ba81c538372b6f128ed8b285fbc
	ac_follow_object_ptr_ 	= new ActionClientExtended<actor_sim_action::FollowObjectAction, 	actor_sim_action::FollowObjectActionFeedbackConstPtr>	(actor_ns_name + "/action/follow_object", 	nh_ptr);
	ac_lie_down_ptr_ 		= new ActionClientExtended<actor_sim_action::LieDownAction, 		actor_sim_action::LieDownActionFeedbackConstPtr>		(actor_ns_name + "/action/lie_down", 		nh_ptr);
	ac_lie_down_name_ptr_ 	= new ActionClientExtended<actor_sim_action::LieDownNameAction, 	actor_sim_action::LieDownNameActionFeedbackConstPtr>	(actor_ns_name + "/action/lie_down_name", 	nh_ptr);
	ac_set_goal_ptr_ 		= new ActionClientExtended<actor_sim_action::SetGoalAction, 		actor_sim_action::SetGoalActionFeedbackConstPtr>		(actor_ns_name + "/action/set_goal", 		nh_ptr);
	ac_set_goal_name_ptr_ 	= new ActionClientExtended<actor_sim_action::SetGoalNameAction, 	actor_sim_action::SetGoalNameActionFeedbackConstPtr>	(actor_ns_name + "/action/set_goal_name", 	nh_ptr);

}

// ---------------------------------------------------------------------------------

void ActorAction::followObject(const std::string &name) {

	actor_sim_action::FollowObjectGoal goal;
	goal.object_name = name;
	ac_follow_object_ptr_->sendGoal(goal);

}

// ---------------------------------------------------------------------------------

void ActorAction::lieDown(const double &x, const double &y, const double &z, const double &rotation, const std::string &frame) {

	actor_sim_action::LieDownGoal goal;
	goal.x_pos = x;
	goal.y_pos = y;
	goal.z_pos = z;
	goal.rotation = rotation;
	goal.frame = frame;
	ac_lie_down_ptr_->sendGoal(goal);

}

// ---------------------------------------------------------------------------------

void ActorAction::lieDownName(const std::string &name_object, const double &lying_height, const double &rotation) {

	actor_sim_action::LieDownNameGoal goal;
	goal.object_name = name_object;
	goal.lying_height = lying_height;
	goal.rotation = rotation;
	ac_lie_down_name_ptr_->sendGoal(goal);
}

// ---------------------------------------------------------------------------------

void ActorAction::setGoal(const double &x, const double &y, const std::string &frame) {

	actor_sim_action::SetGoalGoal goal;
	goal.x_pos = x;
	goal.y_pos = y;
	goal.frame = frame;
	ac_set_goal_ptr_->sendGoal(goal);

}

// ---------------------------------------------------------------------------------

void ActorAction::setGoalName(const std::string &name_object) {

	actor_sim_action::SetGoalNameGoal goal;
	goal.object_name = name_object;
	ac_set_goal_name_ptr_->sendGoal(goal);

}

// ---------------------------------------------------------------------------------
ActionClientExtended<actor_sim_action::FollowObjectAction, actor_sim_action::FollowObjectActionFeedbackConstPtr>*
ActorAction::getClientPtrFollowObject()	{ return (ac_follow_object_ptr_); }
// ---------------------------------------------------------------------------------
ActionClientExtended<actor_sim_action::LieDownAction, actor_sim_action::LieDownActionFeedbackConstPtr>*
ActorAction::getClientPtrLieDown() 		{ return (ac_lie_down_ptr_); }
// ---------------------------------------------------------------------------------
ActionClientExtended<actor_sim_action::LieDownNameAction, actor_sim_action::LieDownNameActionFeedbackConstPtr>*
ActorAction::getClientPtrLieDownName() 	{ return (ac_lie_down_name_ptr_); }
// ---------------------------------------------------------------------------------
ActionClientExtended<actor_sim_action::SetGoalAction, actor_sim_action::SetGoalActionFeedbackConstPtr>*
ActorAction::getClientPtrSetGoal() 		{ return (ac_set_goal_ptr_); }
// ---------------------------------------------------------------------------------
ActionClientExtended<actor_sim_action::SetGoalNameAction, actor_sim_action::SetGoalNameActionFeedbackConstPtr>*
ActorAction::getClientPtrSetGoalName() 	{ return (ac_set_goal_name_ptr_); }
// ---------------------------------------------------------------------------------

ActorAction::~ActorAction() {
	delete(ac_follow_object_ptr_);
	delete(ac_lie_down_ptr_);
	delete(ac_lie_down_name_ptr_);
	delete(ac_set_goal_ptr_);
	delete(ac_set_goal_name_ptr_);
}

// ---------------------------------------------------------------------------------

