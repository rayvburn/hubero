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

// ------------------------------------------------------------------- //

Connection::Connection(): nh_ptr_(nullptr) // , actor_ptr_(nullptr)
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

	srv_set_goal_ = nh_ptr_->advertiseService(namespace_ + "/SetGoal", &Connection::srvSetGoalCallback, this);
//	srv_set_goal_name_;
//	srv_set_stance_;
//	srv_follow_object_;
//	srv_stop_following_;
//	srv_get_velocity_;
}

// ------------------------------------------------------------------- //


bool Connection::srvSetGoalCallback(actor_sim_srv::SetGoal::Request &req, actor_sim_srv::SetGoal::Response &resp) {

	std::cout << "\n\n\nsrvSetGoalCallback()\n\n\n" << std::endl;
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
/*
void Connection::srvSetGoalNameCallback(const actor_sim_srv::SetGoalNameRequestConstPtr &req, actor_sim_srv::SetGoalNameResponsePtr &resp) {

	std::cout << "\n\n\nsrvSetGoalNameCallback()\n\n\n" << std::endl;
//	if ( actor_ptr_->setNewTarget(req->object_name) ) {
//		resp->flag = true;
//	} else {
//		resp->flag = false;
//	}

}

// ------------------------------------------------------------------- //

void Connection::srvSetStanceCallback(const actor_sim_srv::SetStanceRequestConstPtr &req, actor_sim_srv::SetStanceResponsePtr &resp) {

	std::cout << "\n\n\nsrvSetStanceCallback()\n\n\n" << std::endl;
//	if ( actor_ptr_->setStance(static_cast<actor::ActorStance>(req->stance_type_enum)) ) {
//		resp->flag = true;
//	} else {
//		resp->flag = false;
//	}

}

// ------------------------------------------------------------------- //

void Connection::srvFollowObjectCallback(const actor_sim_srv::FollowObjectRequestConstPtr &req, actor_sim_srv::FollowObjectResponsePtr &resp) {

	std::cout << "\n\n\nsrvFollowObjectCallback()\n\n\n" << std::endl;
//	resp->resp_id = 0;

}

// ------------------------------------------------------------------- //

void Connection::srvStopFollowingCallback(const actor_sim_srv::StopFollowingRequestConstPtr &req, actor_sim_srv::StopFollowingResponsePtr &resp) {

	std::cout << "\n\n\nsrvStopFollowingCallback()\n\n\n" << std::endl;
//	resp->flag = 0;

}

// ------------------------------------------------------------------- //

void Connection::srvGetVelocityCallback(const actor_sim_srv::GetVelocityRequestConstPtr &req, actor_sim_srv::GetVelocityResponsePtr &resp) {

	std::cout << "\n\n\nsrvGetVelocityCallback()\n\n\n" << std::endl;
//	std::array<double, 3> velocity = actor_ptr_->getVelocity();
//	resp->x = velocity.at(0);
//	resp->y = velocity.at(1);
//	resp->yaw = velocity.at(2);

}
*/

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
