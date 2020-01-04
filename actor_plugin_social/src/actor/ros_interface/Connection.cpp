/*
 * Connection.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#include <actor/core/Enums.h>
#include <actor/ros_interface/Connection.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <sfm/core/SFMDebug.h>
#include <actor/FrameGlobal.h>

#include <boost/bind.hpp>
#include <boost/phoenix/bind/bind_member_function.hpp>

#include <chrono>
#include <actor/core/Action.h>

// - - - - - - - - - - - - - - - -

namespace actor {
namespace ros_interface {

// declaration of a static variable(s) - database is shared among actors
incare::communication::RobotCommands Connection::voice_robot_;

// ------------------------------------------------------------------- //

Connection::Connection() {

	// create a TfListener instance
	tf_listener_ptr_ = std::unique_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(5.1)));

	action_follow_object_ptr_ = nullptr;

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

	srv_communicate_	= nh_ptr_->advertiseService(namespace_ + "/communicate", 	&Connection::srvCommunicateCallback, this);

	// Verbal interface
	std::cout << "\n\nVOICE TEST\n\n\n";
	for (size_t i = 0; i < voice_robot_.getDatabase().size(); i++) {
		for (size_t j = 0; j < voice_robot_.getDatabase().at(i).size(); j++) {
			std::cout << voice_robot_.getDatabase().at(i).at(j) << "\t";
		}
		std::cout << "\n";
	}
	std::cout << "\n\n\nEND\n\n";

	// ---------------
	// actions section
	action_follow_object_ptr_ = new actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>(
			*nh_ptr_, namespace_ + "/action/follow_object", boost::bind(&Connection::actionFollowObjectCallback, this, _1), false);
	action_follow_object_ptr_->start();
	// ---------------

}

// ------------------------------------------------------------------- //

void Connection::startCallbackProcessingThread() {

	// first check if NodeHandle pointer was set
	if ( (nh_ptr_ == nullptr) || (actor_ptr_.expired()) ) {
		std::cout << "\n\n\nConnection::startCallbackProcessingThread() - NodeHandle Null or ActorPtr expired!()\n\n\n";
		return;
	}

	nh_ptr_->setCallbackQueue(&cb_queue_);
	callback_srv_thread_ = std::thread( std::bind(&Connection::callbackThreadHandler, this) );

}

// ------------------------------------------------------------------- //

//bool Connection::prepareForTeleoperation() {
//
//	// first check if NodeHandle pointer was set
//	if ( nh_ptr_ == nullptr ) {
//		return (false);
//	}
//	// no thread termination possibility (unless !ros::ok() )
//	callback_thread_ = std::thread( std::bind(&Connection::callbackThreadHandler, this) );
//
//	// version with thread termination possibility
////	callback_thread_ = std::thread( std::bind(&Connection::callbackThreadHandler, this), std::move(future_obj_) );
//
//	return (true);
//
//}

// ------------------------------------------------------------------- //

bool Connection::srvSetGoalCallback(actor_sim_srv::SetGoal::Request &req, actor_sim_srv::SetGoal::Response &resp) {

	std::cout << "\nsrvSetGoalCallback()" << "\t" << namespace_ << "\n" << std::endl;

	// Preprocessing step - assert point expressed in the `world` frame;
	// Whatever frame the goal point is expressed in, it will be converted
	// into ACTOR_GLOBAL_FRAME_ID.
	// NOTE: all internal containers (path, target) use the ACTOR_GLOBAL_FRAME_ID
	// as a reference (i.e. store points expressed in ACTOR_GLOBAL_FRAME_ID).
	geometry_msgs::PoseStamped pose_global;

	// this is not necessary if point is defined in the actor global frame
	tf_listener_ptr_->transformPose(actor::FrameGlobal::getFrame(),
									actor::ros_interface::Conversion::convertIgnVectorToPoseStamped(ignition::math::Vector3d(req.x_pos, req.y_pos, 0.0), req.frame),
									pose_global);

	ignition::math::Pose3d pose;
	pose.Pos().X() = static_cast<double>(pose_global.pose.position.x);
	pose.Pos().Y() = static_cast<double>(pose_global.pose.position.y);

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

	} else {

		SfmSetPrintData(false);

	}

	resp.success = true;
	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvCommunicateCallback(incare_human_robot_communication::Communicate::Request &req, incare_human_robot_communication::Communicate::Response &resp) {

	std::cout << "\nsrvCommunicateCallback()" << "\t" << namespace_ << "\n" << std::endl;

	//	req.str

	//	std::vector<std::string> db;
//	for (size_t i = 0; i < db.size(); i++) {
//		db.at(i)
//	}
//	if (s1.find(s2) != std::string::npos) {
//
//	}

	resp.str = "TEST_RESP";
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

void Connection::actionFollowObjectCallback(const actor_sim_action::FollowObjectGoalConstPtr &goal) {

	std::cout << "\nactionFollowObjectCallback()" << "\t" << namespace_ << "\n" << std::endl;

	actor_sim_action::FollowObjectFeedback feedback;
	actor_sim_action::FollowObjectResult result;

	// take ownership of the Actor shared_ptr
	if ( !actor_ptr_.lock()->followObject(goal->object_name) ) {
		std::cout << "FollowObject request cannot be processed" << std::endl;
		result.status = 0; // actor::core::Action::FAILED
		result.text = "'FollowObject' action request cannot be processed";
		action_follow_object_ptr_->setAborted(result, "RESULT ABORTED");
		return;
	}

	while ( !actor_ptr_.lock()->getActionInfo().isTerminated() ) {
		feedback.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
		feedback.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
		action_follow_object_ptr_->publishFeedback(feedback);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// FINISH
	result.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	result.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();

	action_follow_object_ptr_->setSucceeded(result, "RESULT TEST");

	std::cout << "\nactionFollowObjectCallback()" << "\t" << namespace_ << "\tFINISH\n" << std::endl;

}

// ------------------------------------------------------------------- //

Connection::~Connection() {}

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
