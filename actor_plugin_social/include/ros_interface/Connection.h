/*
 * Connection.h
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ROS_INTERFACE_CONNECTION_H_
#define INCLUDE_ROS_INTERFACE_CONNECTION_H_

// C++ STL
#include <memory> // std::shared_ptr
#include <thread>
#include <string>

// ROS headers
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/callback_queue.h>

// Actor
//#include <core/Actor.h>

// service definitions
#include <actor_sim_srv/FollowObject.h>
#include <actor_sim_srv/StopFollowing.h>
#include <actor_sim_srv/SetGoal.h>
#include <actor_sim_srv/SetGoalName.h>
#include <actor_sim_srv/SetStance.h>
#include <actor_sim_srv/GetVelocity.h>


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
	/// set() or get() methods on the Actor's object to further pass data
	// TODO: pass shared_ptr
	// circular dependency problem
//	void setActorPtr(std::shared_ptr<actor::core::Actor> actor_ptr);
	//void setActorPtr(actor::core::Actor *actor_ptr);

	/// \brief Advertises services if NodeHandle was previously set
	void initServices();

	/// \brief Default destructor
	virtual ~Connection();

private:

	/// \brief Callbacks for each service
	void srvSetGoalCallback(const actor_sim_srv::SetGoalRequestConstPtr &req, actor_sim_srv::SetGoalResponsePtr &resp);
	void srvSetGoalNameCallback(const actor_sim_srv::SetGoalNameRequestConstPtr &req, actor_sim_srv::SetGoalNameResponsePtr &resp);
	void srvSetStanceCallback(const actor_sim_srv::SetStanceRequestConstPtr &req, actor_sim_srv::SetStanceResponsePtr &resp);
	void srvFollowObjectCallback(const actor_sim_srv::FollowObjectRequestConstPtr &req, actor_sim_srv::FollowObjectResponsePtr &resp);
	void srvStopFollowingCallback(const actor_sim_srv::StopFollowingRequestConstPtr &req, actor_sim_srv::StopFollowingResponsePtr &resp);
	void srvGetVelocityCallback(const actor_sim_srv::GetVelocityRequestConstPtr &req, actor_sim_srv::GetVelocityResponsePtr &resp);

	/// \brief `main` for a callback thread
	void callbackThreadHandler();

	/// \brief NodeHandle object to provide communication with external ROS systems
	std::shared_ptr<ros::NodeHandle> nh_ptr_;

	/// \brief Namespace in which all services will be available (actor's name)
	std::string namespace_;

	/// \brief Pointer to an Actor object
	//std::shared_ptr<actor::core::Actor> actor_ptr_;
	//actor::core::Actor *actor_ptr_;

	/// \brief ROS Callback queue to process messages
	ros::CallbackQueue cb_queue_;

	/// \brief Separate thread to provide a fast response to a service call
	std::thread callback_thread_;

	/// \brief Servers for each service
	ros::ServiceServer srv_set_goal_;
	ros::ServiceServer srv_set_goal_name_;
	ros::ServiceServer srv_set_stance_;
	ros::ServiceServer srv_follow_object_;
	ros::ServiceServer srv_stop_following_;
	ros::ServiceServer srv_get_velocity_;

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_CONNECTION_H_ */
