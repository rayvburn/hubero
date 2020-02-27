/*
 * ActorAPI.h
 *
 *  Created on: Feb 24, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_SIM_API_ACTORAPI_H_
#define INCLUDE_ACTOR_SIM_API_ACTORAPI_H_

#include <ros/ros.h>
#include <actor_sim_srv/GetPose.h>
#include <actor_sim_srv/GetVelocity.h>
#include <actor_sim_srv/SetStance.h>
#include <std_srvs/Trigger.h>

class ActorAPI {

public:

	/// \brief Parametrized constructor
	ActorAPI(const std::string &actor_ns_name, ros::NodeHandle *nh_ptr);

	bool moveAroundStart();
	bool moveAroundStop();

	bool setStance(const uint8_t &stance_type);

	/// \brief Get velocity vector (linear x, linear y and angular `yaw`)
	std::array<double, 3> getVelocity(); // cannot be const

	/// \brief Returns pose vector (3 translation components
	/// and 3 rotation components (roll, pitch and yaw) expressed
	/// in the world coordinate system (which is rotated relative
	/// to the actor coordinate system)
	std::array<double, 6> getPose(const std::string &frame = "world");

	/// \brief Default destructor
	virtual ~ActorAPI();

protected:

	/// \brief `get_velocity` service client (`sc`)
	ros::ServiceClient sc_get_velocity_;

	/// \brief `get_pose` service client (`sc`)
	ros::ServiceClient sc_get_pose_;

	/// \brief `move_around_start` service client (`sc`)
	ros::ServiceClient sc_move_around_start_;

	/// \brief `move_around_stop` service client (`sc`)
	ros::ServiceClient sc_move_around_stop_;

	/// \brief `set_stance` service client (`sc`)
	ros::ServiceClient sc_set_stance_;

};

#endif /* INCLUDE_ACTOR_SIM_API_ACTORAPI_H_ */
