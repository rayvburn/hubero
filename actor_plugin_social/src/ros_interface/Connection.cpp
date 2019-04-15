/*
 * Connection.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#include "ros_interface/Connection.h"

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "core/Enums.h"


namespace actor {
namespace ros_interface {

// TODO: start callback thread only when NH and actor are initialized!

// ------------------------------------------------------------------- //

Connection::Connection()
{
	// TODO Auto-generated constructor stub

}

// ------------------------------------------------------------------- //

void Connection::setNodeHandle(std::shared_ptr<ros::NodeHandle> nh_ptr) {
	nh_ptr_ = nh_ptr;
}

// ------------------------------------------------------------------- //
void Connection::setNamespace(const std::string &ns) {
	namespace_ = ns;
}

// ------------------------------------------------------------------- //

void Connection::setActorPtr(std::shared_ptr<actor::core::Actor> actor_ptr) {
	actor_ptr_ = actor_ptr;
}

// ------------------------------------------------------------------- //

void Connection::initServices() {

	// check if NodeHandle pointer was set
	if ( nh_ptr_ == nullptr ) {
		return;
	}

	srv_set_goal_ 		= nh_ptr_->advertiseService(namespace_ + "/set_goal", 		&Connection::srvSetGoalCallback, this);
	srv_set_goal_name_ 	= nh_ptr_->advertiseService(namespace_ + "/set_goal_name", 	&Connection::srvSetGoalNameCallback, this);
	srv_set_stance_ 	= nh_ptr_->advertiseService(namespace_ + "/set_stance", 	&Connection::srvSetStanceCallback, this);
	srv_follow_object_ 	= nh_ptr_->advertiseService(namespace_ + "/follow_object", 	&Connection::srvFollowObjectCallback, this);
	srv_stop_following_ = nh_ptr_->advertiseService(namespace_ + "/stop_following", &Connection::srvStopFollowingCallback, this);
	srv_get_velocity_ 	= nh_ptr_->advertiseService(namespace_ + "/get_velocity", 	&Connection::srvGetVelocityCallback, this);

}

// ------------------------------------------------------------------- //

bool Connection::srvSetGoalCallback(actor_sim_srv::SetGoal::Request &req, actor_sim_srv::SetGoal::Response &resp) {

	std::cout << "\n\n\nsrvSetGoalCallback()" << "\t" << namespace_ << "\n\n\n" << std::endl;
//	ignition::math::Pose3d pose;
//	pose.Pos().X() = static_cast<double>(req->x_pos);
//	pose.Pos().Y() = static_cast<double>(req->y_pos);
//	//actor_ptr_->setNewTarget(pose);
//
//	// response's flag does not matter
//	resp->flag = true;

	return true;

}

// ------------------------------------------------------------------- //

bool Connection::srvSetGoalNameCallback(actor_sim_srv::SetGoalName::Request &req, actor_sim_srv::SetGoalName::Response &resp) {

	std::cout << "\n\n\nsrvSetGoalNameCallback()" << "\t" << namespace_ << "\n\n\n" << std::endl;
//	if ( actor_ptr_->setNewTarget(req->object_name) ) {
//		resp->flag = true;
//	} else {
//		resp->flag = false;
//	}

	return true;

}

// ------------------------------------------------------------------- //

bool Connection::srvSetStanceCallback(actor_sim_srv::SetStance::Request &req, actor_sim_srv::SetStance::Response &resp) {

	std::cout << "\n\n\nsrvSetStanceCallback()" << "\t" << namespace_ << "\n\n\n" << std::endl;
//	if ( actor_ptr_->setStance(static_cast<actor::ActorStance>(req->stance_type_enum)) ) {
//		resp->flag = true;
//	} else {
//		resp->flag = false;
//	}

	return true;

}

// ------------------------------------------------------------------- //

bool Connection::srvFollowObjectCallback(actor_sim_srv::FollowObject::Request &req, actor_sim_srv::FollowObject::Response &resp) {

	std::cout << "\n\n\nsrvFollowObjectCallback()" << "\t" << namespace_ << "\n\n\n" << std::endl;
//	resp->resp_id = 0;

	return true;

}

// ------------------------------------------------------------------- //

bool Connection::srvStopFollowingCallback(actor_sim_srv::StopFollowing::Request &req, actor_sim_srv::StopFollowing::Response &resp) {

	std::cout << "\n\n\nsrvStopFollowingCallback()" << "\t" << namespace_ << "\n\n\n" << std::endl;
//	resp->flag = 0;

	return true;

}

// ------------------------------------------------------------------- //

bool Connection::srvGetVelocityCallback(actor_sim_srv::GetVelocity::Request &req, actor_sim_srv::GetVelocity::Response &resp) {

	std::cout << "\n\n\nsrvGetVelocityCallback()" << "\t" << namespace_ << "\n\n\n" << std::endl;
//	std::array<double, 3> velocity = actor_ptr_->getVelocity();
//	resp->x = velocity.at(0);
//	resp->y = velocity.at(1);
//	resp->yaw = velocity.at(2);

	return true;

}


// ------------------------------------------------------------------- //

void Connection::callbackThreadHandler() {

}

// ------------------------------------------------------------------- //

Connection::~Connection() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
