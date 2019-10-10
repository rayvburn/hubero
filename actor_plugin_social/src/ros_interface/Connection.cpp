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

// - - - - - - - - - - - - - - - -
// SFM debug switcher
#include "sfm/core/SFMDebug.h"
#include "BoundingEllipseDebug.h"
// - - - - - - - - - - - - - - - -

namespace actor {
namespace ros_interface {

// ------------------------------------------------------------------- //

Connection::Connection()
{

	// threading - get std::future object out of std::promise
//	future_obj_ = exit_signal_.get_future();

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

	// first check if NodeHandle pointer was set
	if ( (nh_ptr_ == nullptr) || (actor_ptr_.expired()) ) {
		std::cout << "\n\n\nConnection::initServices() - NodeHandle Null or ActorPtr expired!()\n\n\n";
		return;
	}

	srv_set_goal_ 		= nh_ptr_->advertiseService(namespace_ + "/set_goal", 		&Connection::srvSetGoalCallback, this);
	srv_set_goal_name_ 	= nh_ptr_->advertiseService(namespace_ + "/set_goal_name", 	&Connection::srvSetGoalNameCallback, this);
	srv_set_stance_ 	= nh_ptr_->advertiseService(namespace_ + "/set_stance", 	&Connection::srvSetStanceCallback, this);
	srv_follow_object_ 	= nh_ptr_->advertiseService(namespace_ + "/follow_object", 	&Connection::srvFollowObjectCallback, this);
	srv_stop_following_ = nh_ptr_->advertiseService(namespace_ + "/stop_following", &Connection::srvStopFollowingCallback, this);
	srv_get_velocity_ 	= nh_ptr_->advertiseService(namespace_ + "/get_velocity", 	&Connection::srvGetVelocityCallback, this);
	srv_lie_down_ 		= nh_ptr_->advertiseService(namespace_ + "/lie_down", 		&Connection::srvLieDownCallback, this);
	srv_lie_down_name_	= nh_ptr_->advertiseService(namespace_ + "/lie_down_name",	&Connection::srvLieDownNameCallback, this);
	srv_lie_down_stop_	= nh_ptr_->advertiseService(namespace_ + "/lie_down_stop",	&Connection::srvLieDownStopCallback, this);

	srv_switch_debug_sfm_= nh_ptr_->advertiseService(namespace_ + "/debug_sfm", 	&Connection::srvSetDebugSFMCallback, this);

}

// ------------------------------------------------------------------- //

void Connection::startCallbackProcessingThread() {

	// first check if NodeHandle pointer was set
	if ( (nh_ptr_ == nullptr) || (actor_ptr_.expired()) ) {
		std::cout << "\n\n\nConnection::startCallbackProcessingThread() - NodeHandle Null or ActorPtr expired!()\n\n\n";
		return;
	}

	nh_ptr_->setCallbackQueue(&cb_queue_);
	callback_thread_ = std::thread( std::bind(&Connection::callbackThreadHandler, this) );

}

// ------------------------------------------------------------------- //

bool Connection::prepareForTeleoperation() {

	// first check if NodeHandle pointer was set
	if ( nh_ptr_ == nullptr ) {
		return (false);
	}
	// no thread termination possibility (unless !ros::ok() )
	callback_thread_ = std::thread( std::bind(&Connection::callbackThreadHandler, this) );

	// version with thread termination possibility
//	callback_thread_ = std::thread( std::bind(&Connection::callbackThreadHandler, this), std::move(future_obj_) );

	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvSetGoalCallback(actor_sim_srv::SetGoal::Request &req, actor_sim_srv::SetGoal::Response &resp) {

	std::cout << "\nsrvSetGoalCallback()" << "\t" << namespace_ << "\n" << std::endl;
	ignition::math::Pose3d pose;
	pose.Pos().X() = static_cast<double>(req.x_pos);
	pose.Pos().Y() = static_cast<double>(req.y_pos);

	// take ownership of the Actor shared_ptr
	resp.flag = actor_ptr_.lock()->setNewTarget(pose);

	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvSetGoalNameCallback(actor_sim_srv::SetGoalName::Request &req, actor_sim_srv::SetGoalName::Response &resp) {

	std::cout << "\nsrvSetGoalNameCallback()" << "\t" << namespace_ << "\n" << std::endl;

	// take ownership of the Actor shared_ptr
	resp.flag = actor_ptr_.lock()->setNewTarget(req.object_name);

	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvSetStanceCallback(actor_sim_srv::SetStance::Request &req, actor_sim_srv::SetStance::Response &resp) {

	std::cout << "\nsrvSetStanceCallback()" << "\t" << namespace_ << "\n" << std::endl;

	// take ownership of the Actor shared_ptr
	resp.flag = actor_ptr_.lock()->setStance(static_cast<actor::ActorStance>(req.stance_enum));

	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvFollowObjectCallback(actor_sim_srv::FollowObject::Request &req, actor_sim_srv::FollowObject::Response &resp) {

	std::cout << "\nsrvFollowObjectCallback()" << "\t" << namespace_ << "\n" << std::endl;

	// take ownership of the Actor shared_ptr
	resp.flag = actor_ptr_.lock()->followObject(req.object_name);

	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvStopFollowingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

	std::cout << "\nsrvStopFollowingCallback()" << "\t" << namespace_ << "\n" << std::endl;

	// take ownership of the Actor shared_ptr
	resp.success = actor_ptr_.lock()->followObjectStop();

	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvGetVelocityCallback(actor_sim_srv::GetVelocity::Request &req, actor_sim_srv::GetVelocity::Response &resp) {

	std::cout << "\nsrvGetVelocityCallback()" << "\t" << namespace_ << "\n" << std::endl;

	// take ownership of the Actor shared_ptr
	std::array<double, 3> velocity = actor_ptr_.lock()->getVelocity();
	resp.x = velocity.at(0);
	resp.y = velocity.at(1);
	resp.yaw = velocity.at(2);

	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvLieDownCallback(actor_sim_srv::LieDown::Request &req, actor_sim_srv::LieDown::Response &resp) {

	std::cout << "\nsrvLieDownCallback()" << "\t" << namespace_ << "\n" << std::endl;
	// take ownership of the Actor shared_ptr
	resp.flag = actor_ptr_.lock()->lieDown(req.x_pos, req.y_pos, req.z_pos, req.rotation);
	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvLieDownNameCallback(actor_sim_srv::LieDownName::Request	&req, actor_sim_srv::LieDownName::Response &resp) {

	std::cout << "\nsrvLieDownNameCallback()" << "\t" << namespace_ << "\n" << std::endl;
	// take ownership of the Actor shared_ptr
	resp.flag = actor_ptr_.lock()->lieDown(req.object_name, req.lying_height, req.rotation);
	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvLieDownStopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

	std::cout << "\nsrvLieDownStopCallback()" << "\t" << namespace_ << "\n" << std::endl;
	// take ownership of the Actor shared_ptr
	resp.success = actor_ptr_.lock()->lieDownStop();
	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvSetDebugSFMCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp) {

	std::cout << "\nsrvSetDebugSFMCallback()" << "\t" << namespace_ << "\n" << std::endl;
	if ( req.data == true ) {

		SfmSetPrintData(true);
		debugEllipseSet(true);

	} else {

		SfmSetPrintData(false);
		debugEllipseSet(false);

	}

	resp.success = true;
	return (true);

}

// ------------------------------------------------------------------- //

void Connection::callbackThreadHandler() {

	// thread calling message callbacks for those which wait in the queue
	while ( nh_ptr_->ok() ) {

		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		cb_queue_.callAvailable(); //  ros::WallDuration(0.001)

	}

}

// ------------------------------------------------------------------- //

Connection::~Connection() { }

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
