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

	sc_move_around_start_ = nh_ptr->serviceClient<std_srvs::Trigger>(actor_ns_name + "/move_around");
	sc_move_around_stop_ = nh_ptr->serviceClient<std_srvs::Trigger>(actor_ns_name + "/move_around_stop");
	sc_set_stance_ = nh_ptr->serviceClient<actor_sim_srv::SetStance>(actor_ns_name + "/set_stance");

}

// ---------------------------------------------------------------------------------

bool ActorAPI::moveAroundStart() {

	std_srvs::Trigger srv;
	return (sc_move_around_start_.call(srv));

}

// ---------------------------------------------------------------------------------

bool ActorAPI::moveAroundStop() {

	std_srvs::Trigger srv;
	return (sc_move_around_stop_.call(srv));

}

// ---------------------------------------------------------------------------------

bool ActorAPI::setStance(const uint8_t &stance_type) {

	actor_sim_srv::SetStance srv;
	srv.request.stance_enum = stance_type;
	return (sc_set_stance_.call(srv));

}

// ---------------------------------------------------------------------------------

bool ActorAPI::setStance(const actor::ActorStance &stance_type) {
	return (setStance(static_cast<uint8_t>(stance_type)));
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
