/*
 * Connection.h
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_ROS_INTERFACE_CONNECTION_H_
#define INCLUDE_ACTOR_ROS_INTERFACE_CONNECTION_H_

// C++ STL
#include <actor/core/Actor.h>
#include <actor/core/ActorFwd.h> 	// must be here due to the circular dependency
#include <memory> 					// std::shared_ptr, std::unique_ptr
#include <thread>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>	// ROS Kinetic

// Actor services
#include <actor_sim_srv/FollowObject.h>
#include <actor_sim_srv/SetGoal.h>
#include <actor_sim_srv/SetGoalName.h>
#include <actor_sim_srv/SetStance.h>
#include <actor_sim_srv/GetVelocity.h>
#include <actor_sim_srv/LieDown.h>
#include <actor_sim_srv/LieDownName.h>
#include <actor_sim_srv/GetPose.h>

#include <actor/ros_interface/Conversion.h>

#include <std_srvs/Trigger.h>	// stopLying, stopFollowing
#include <std_srvs/SetBool.h>	// switching on/off SFM's debugging info

// Actor actions
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actor_sim_action/FollowObjectAction.h>
#include <actor_sim_action/SetGoalAction.h>
#include <actor_sim_action/SetGoalNameAction.h>
#include <actor_sim_action/LieDownAction.h>
#include <actor_sim_action/LieDownNameAction.h>

namespace actor {
namespace ros_interface {

class Connection {

public:

	/// \brief Default constructor
	Connection();

	/// \brief Loads a ros::NodeHandle pointer;
	/// A shared node is used for communication with ROS
	/// to avoid creating a separate node for each Actor
	void setNodeHandle(std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Sets a namespace within all topics will be published;
	/// Usually namespace will be the actor's name
	void setNamespace(const std::string &ns);

	/// \brief Sets pointer to an Actor as there is a need to invoke some
	/// set() or get() methods on the Actor's object to further pass data -
	/// circular dependency
	void setActorPtr(std::shared_ptr<actor::core::Actor> actor_ptr);

	/// \brief Advertises services if NodeHandle was previously set
	void initServices();

	/// \brief Starts action servers if the NodeHandle was previously set
	void initActions();

	/// \brief Sets callback queue and starts a new thread
	/// for services' callbacks processing
	void startCallbackProcessingThread();

//	/// \brief Initializes callback queue, starts separate thread
//	/// for callback handling, used for teleoperation mode of an Actor
//	/// DEPRECATED
//	bool prepareForTeleoperation();

//	// TODO: makes sense when thread termination is possible
//	/// \brief Invoked after teleoperation mode stoppage
//	/// DEPRECATED
//	void finishTeleoperation();

	/// \brief Default destructor
	virtual ~Connection();

private:

	/// \brief Transforms the point with `x_pos` and `y_pos` coordinates expressed in `frame`
	/// coordinate system to the actor global coordinate system (usually `world`)
	ignition::math::Pose3d transformPoint(const std::string &frame, const double &x_pos, const double &y_pos, const double &z_pos = 0.0);

	/// \brief Callbacks for each service, service callback must return bool
	bool srvSetGoalCallback			(actor_sim_srv::SetGoal::Request		&req,	actor_sim_srv::SetGoal::Response		&resp);
	bool srvSetGoalNameCallback		(actor_sim_srv::SetGoalName::Request 	&req,	actor_sim_srv::SetGoalName::Response 	&resp);
	bool srvSetStanceCallback		(actor_sim_srv::SetStance::Request 		&req,	actor_sim_srv::SetStance::Response 		&resp);
	bool srvFollowObjectCallback	(actor_sim_srv::FollowObject::Request 	&req, 	actor_sim_srv::FollowObject::Response 	&resp);
	bool srvStopFollowingCallback	(std_srvs::Trigger::Request 			&req, 	std_srvs::Trigger::Response 			&resp);
	bool srvGetVelocityCallback		(actor_sim_srv::GetVelocity::Request 	&req,	actor_sim_srv::GetVelocity::Response 	&resp);
	bool srvLieDownCallback			(actor_sim_srv::LieDown::Request		&req,	actor_sim_srv::LieDown::Response 		&resp);
	bool srvLieDownNameCallback		(actor_sim_srv::LieDownName::Request	&req,	actor_sim_srv::LieDownName::Response 	&resp);
	bool srvLieDownStopCallback		(std_srvs::Trigger::Request				&req,	std_srvs::Trigger::Response 			&resp);
	bool srvMoveAroundCallback		(std_srvs::Trigger::Request 			&req, 	std_srvs::Trigger::Response 			&resp);
	bool srvMoveAroundStopCallback	(std_srvs::Trigger::Request 			&req, 	std_srvs::Trigger::Response 			&resp);
	bool srvGetPoseCallback			(actor_sim_srv::GetPose::Request 		&req,	actor_sim_srv::GetPose::Response 		&resp);

	/// \brief Switcher of a debug info printing
	bool srvSetDebugSFMCallback		(std_srvs::SetBool::Request				&req,	std_srvs::SetBool::Response 			&resp);

	/// \brief `main` for a callback thread
	void callbackThreadHandler();

	/// \brief NodeHandle object to provide communication with external ROS systems
	/// nullptr by default
	std::shared_ptr<ros::NodeHandle> nh_ptr_;

	/// \brief Transform listener, used for the conversion from the goal frame
	/// to the actor global frame (associated with the Gazebo world center).
	std::unique_ptr<tf::TransformListener> tf_listener_ptr_;

	/// \brief Namespace in which all services will be available (actor's name)
	std::string namespace_;

	/// \brief Pointer to an Actor object
	/// nullptr by default
	std::weak_ptr<actor::core::Actor> actor_ptr_;

	/// \brief ROS Callback queue to process messages;
	/// typical pointer just like NodeHandle requires
	ros::CallbackQueue cb_queue_;

	/// \brief Separate thread to provide a fast response to a service call
	std::thread callback_srv_thread_;

//	// Reference: https://thispointer.com/c11-how-to-stop-or-terminate-a-thread/
//	/// \brief Object used as a helper to terminate a thread
//	std::promise<void> exit_signal_;
//
//	/// \brief Object used to terminate a thread
//	std::future<void> future_obj_;

	/// \brief Servers for each service
	ros::ServiceServer srv_set_goal_;
	ros::ServiceServer srv_set_goal_name_;
	ros::ServiceServer srv_set_stance_;
	ros::ServiceServer srv_follow_object_;
	ros::ServiceServer srv_stop_following_;
	ros::ServiceServer srv_get_velocity_;
	ros::ServiceServer srv_lie_down_;
	ros::ServiceServer srv_lie_down_name_;
	ros::ServiceServer srv_lie_down_stop_;
	ros::ServiceServer srv_move_around_;
	ros::ServiceServer srv_move_around_stop_;
	ros::ServiceServer srv_get_pose_;

	/// \brief Only for debugging purposes
	ros::ServiceServer srv_switch_debug_sfm_;

	/// \section Actions
	/// \brief
	/// \note See http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29#The_Code
	/// "NodeHandle instance must be created before this line. Otherwise strange error occurs." - thus a pointer created
	actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>* action_follow_object_ptr_;
	actionlib::SimpleActionServer<actor_sim_action::SetGoalAction>* action_set_goal_ptr_;
	actionlib::SimpleActionServer<actor_sim_action::SetGoalNameAction>* action_set_goal_name_ptr_;
	actionlib::SimpleActionServer<actor_sim_action::LieDownAction>* action_lie_down_ptr_;
	actionlib::SimpleActionServer<actor_sim_action::LieDownNameAction>* action_lie_down_name_ptr_;
	/// \brief Callbacks for each action
	void actionFollowObjectCallback(const actor_sim_action::FollowObjectGoalConstPtr &goal);
	void actionSetGoalCallback(const actor_sim_action::SetGoalGoalConstPtr &goal);
	void actionSetGoalNameCallback(const actor_sim_action::SetGoalNameGoalConstPtr &goal);
	void actionLieDownCallback(const actor_sim_action::LieDownGoalConstPtr &goal);
	void actionLieDownNameCallback(const actor_sim_action::LieDownNameGoalConstPtr &goal);

	/// \note The template method cannot be created - `invalid use of incomplete type`
	/// \section FollowObjectAction
	void publishAbort(actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>* as_ptr);
	void publishFeedback(actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>* as_ptr);
	void publishResult(actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>* as_ptr);
	/// \section SetGoal
	void publishAbort(actionlib::SimpleActionServer<actor_sim_action::SetGoalAction>* as_ptr);
	void publishFeedback(actionlib::SimpleActionServer<actor_sim_action::SetGoalAction>* as_ptr);
	void publishResult(actionlib::SimpleActionServer<actor_sim_action::SetGoalAction>* as_ptr);
	/// \section SetGoalName
	void publishAbort(actionlib::SimpleActionServer<actor_sim_action::SetGoalNameAction>* as_ptr);
	void publishFeedback(actionlib::SimpleActionServer<actor_sim_action::SetGoalNameAction>* as_ptr);
	void publishResult(actionlib::SimpleActionServer<actor_sim_action::SetGoalNameAction>* as_ptr);
	/// \section LieDown
	void publishAbort(actionlib::SimpleActionServer<actor_sim_action::LieDownAction>* as_ptr);
	void publishFeedback(actionlib::SimpleActionServer<actor_sim_action::LieDownAction>* as_ptr);
	void publishResult(actionlib::SimpleActionServer<actor_sim_action::LieDownAction>* as_ptr);
	/// \section LieDownName
	void publishAbort(actionlib::SimpleActionServer<actor_sim_action::LieDownNameAction>* as_ptr);
	void publishFeedback(actionlib::SimpleActionServer<actor_sim_action::LieDownNameAction>* as_ptr);
	void publishResult(actionlib::SimpleActionServer<actor_sim_action::LieDownNameAction>* as_ptr);

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_ROS_INTERFACE_CONNECTION_H_ */
