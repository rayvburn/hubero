/*
 * ActorAPI.cpp
 *
 *  Created on: Feb 24, 2020
 *      Author: rayvburn
 */

//#include "../../include/actor_sim_api/ActorAPI.h"
#include <actor_sim_api/ActorAPI.h>

ActorAPI::ActorAPI(const std::string &actor_ns_name, ros::NodeHandle *nh_ptr) {

	sc_get_velocity_ = nh_ptr->serviceClient<actor_sim_srv::GetVelocity>(actor_ns_name + "/get_velocity");
	sc_get_pose_ = nh_ptr->serviceClient<actor_sim_srv::GetVelocity>(actor_ns_name + "/get_pose");

}

// ---------------------------------------------------------------------------------

std::array<double, 3> ActorAPI::getVelocity() {

	actor_sim_srv::GetVelocity srv; // no fields to fill
	std::array<double, 3> resp = {0};

	if ( !sc_get_velocity_.call(srv) ) {
		return (resp);
	}

	resp.at(0) = srv.response.x;
	resp.at(1) = srv.response.y;
	resp.at(2) = srv.response.yaw;

	return (resp);

}

// ---------------------------------------------------------------------------------

std::array<double, 6> ActorAPI::getPose(const std::string &frame) {

	actor_sim_srv::GetPose srv;
	srv.request.frame = frame;

	std::array<double, 6> resp = {0};

	if ( !sc_get_pose_.call(srv) ) {
		return (resp);
	}

	resp.at(0) = srv.response.x;
	resp.at(1) = srv.response.y;
	resp.at(2) = srv.response.z;
	resp.at(3) = srv.response.roll;
	resp.at(4) = srv.response.pitch;
	resp.at(5) = srv.response.yaw;

	return (resp);

}

// ---------------------------------------------------------------------------------

ActorAPI::~ActorAPI() {}

// ---------------------------------------------------------------------------------
