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

Connection::Connection()
	:	action_follow_object_ptr_(nullptr),
		action_set_goal_ptr_(nullptr),
		action_set_goal_name_ptr_(nullptr),
		action_lie_down_ptr_(nullptr),
		action_lie_down_name_ptr_(nullptr)
{

	// create a TfListener instance
	tf_listener_ptr_ = std::unique_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(5.1)));

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
	srv_move_around_	= nh_ptr_->advertiseService(namespace_ + "/move_around",	&Connection::srvMoveAroundCallback, this);
	srv_move_around_stop_= nh_ptr_->advertiseService(namespace_ + "/move_around_stop",	&Connection::srvMoveAroundStopCallback, this);

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

}

// ------------------------------------------------------------------- //

void Connection::initActions() {

	// first check if NodeHandle pointer was set
	if ( (nh_ptr_ == nullptr) || (actor_ptr_.expired()) ) {
		std::cout << "\n\n\nConnection::initServices() - NodeHandle Null or ActorPtr expired!()\n\n\n";
		return;
	}

	// actions section
	action_follow_object_ptr_ = new actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>(
			*nh_ptr_, namespace_ + "/action/follow_object", boost::bind(&Connection::actionFollowObjectCallback, this, _1), false);
	action_follow_object_ptr_->start();

	action_set_goal_ptr_ = new actionlib::SimpleActionServer<actor_sim_action::SetGoalAction>(
			*nh_ptr_, namespace_ + "/action/set_goal", boost::bind(&Connection::actionSetGoalCallback, this, _1), false);
	action_set_goal_ptr_->start();

	action_set_goal_name_ptr_ = new actionlib::SimpleActionServer<actor_sim_action::SetGoalNameAction>(
			*nh_ptr_, namespace_ + "/action/set_goal_name", boost::bind(&Connection::actionSetGoalNameCallback, this, _1), false);
	action_set_goal_name_ptr_->start();

	action_lie_down_ptr_ = new actionlib::SimpleActionServer<actor_sim_action::LieDownAction>(
			*nh_ptr_, namespace_ + "/action/lie_down", boost::bind(&Connection::actionLieDownCallback, this, _1), false);
	action_lie_down_ptr_->start();

	action_lie_down_name_ptr_ = new actionlib::SimpleActionServer<actor_sim_action::LieDownNameAction>(
			*nh_ptr_, namespace_ + "/action/lie_down_name", boost::bind(&Connection::actionLieDownNameCallback, this, _1), false);
	action_lie_down_name_ptr_->start();

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
	// 'transformPoint'

	// take ownership of the Actor shared_ptr
	resp.flag = actor_ptr_.lock()->setNewTarget(transformPoint(req.frame, req.x_pos, req.y_pos));

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

bool Connection::srvMoveAroundCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

	std::cout << "\nsrvMoveAroundCallback()" << "\t" << namespace_ << "\n" << std::endl;
	// take ownership of the Actor shared_ptr
	resp.success = actor_ptr_.lock()->moveAround();
	return (true);

}

// ------------------------------------------------------------------- //

bool Connection::srvMoveAroundStopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {

	std::cout << "\nsrvMoveAroundStopCallback()" << "\t" << namespace_ << "\n" << std::endl;
	// take ownership of the Actor shared_ptr
	resp.success = actor_ptr_.lock()->moveAroundStop();
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
		publishAbort(action_follow_object_ptr_);
		return;
	}

	while ( !actor_ptr_.lock()->getActionInfo().isTerminated() ) {
		publishFeedback(action_follow_object_ptr_);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	publishFeedback(action_follow_object_ptr_);
	publishResult(action_follow_object_ptr_);

	std::cout << "\nactionFollowObjectCallback()" << "\t" << namespace_ << "\tFINISH\n" << std::endl;

}

// ------------------------------------------------------------------- //

void Connection::actionSetGoalCallback(const actor_sim_action::SetGoalGoalConstPtr &goal) {

	std::cout << "\nactionSetGoalCallback()" << "\t" << namespace_ << "\n" << std::endl;

	actor_sim_action::SetGoalFeedback feedback;
	actor_sim_action::SetGoalResult result;

	if ( !actor_ptr_.lock()->setNewTarget(transformPoint(goal->frame, goal->x_pos, goal->y_pos)) ) {
		std::cout << "SetGoal request cannot be processed" << std::endl;
		publishAbort(action_set_goal_ptr_);
		return;
	}

	while ( !actor_ptr_.lock()->getActionInfo().isTerminated() ) {
		publishFeedback(action_set_goal_ptr_);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	// publish the 'result' as a feedback too
	publishFeedback(action_set_goal_ptr_);
	publishResult(action_set_goal_ptr_);

	std::cout << "\nactionSetGoalCallback()" << "\t" << namespace_ << "\tFINISH\n" << std::endl;

}

// ------------------------------------------------------------------- //

void Connection::actionSetGoalNameCallback(const actor_sim_action::SetGoalNameGoalConstPtr &goal) {

	std::cout << "\nactionSetGoalNameCallback()" << "\t" << namespace_ << "\n" << std::endl;

	actor_sim_action::SetGoalNameFeedback feedback;
	actor_sim_action::SetGoalNameResult result;

	if ( !actor_ptr_.lock()->setNewTarget(goal->object_name) ) {
		std::cout << "SetGoalName request cannot be processed" << std::endl;
		publishAbort(action_set_goal_name_ptr_);
		return;
	}

	while ( !actor_ptr_.lock()->getActionInfo().isTerminated() ) {
		publishFeedback(action_set_goal_name_ptr_);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	publishFeedback(action_set_goal_name_ptr_);
	publishResult(action_set_goal_name_ptr_);

	std::cout << "\nactionSetGoalNameCallback()" << "\t" << namespace_ << "\tFINISH\n" << std::endl;

}

// ------------------------------------------------------------------- //

void Connection::actionLieDownCallback(const actor_sim_action::LieDownGoalConstPtr &goal) {

	std::cout << "\nactionLieDownCallback()" << "\t" << namespace_ << "\n" << std::endl;

	actor_sim_action::LieDownFeedback feedback;
	actor_sim_action::LieDownResult result;

	ignition::math::Pose3d pose_world = transformPoint(goal->frame, goal->x_pos, goal->y_pos, goal->z_pos);
	if ( !actor_ptr_.lock()->lieDown(pose_world.Pos().X(), pose_world.Pos().Y(), pose_world.Pos().Z(), goal->rotation) ) {
		std::cout << "LieDown request cannot be processed" << std::endl;
		publishAbort(action_lie_down_ptr_);
		return;
	}

	while ( !actor_ptr_.lock()->getActionInfo().isTerminated() ) {
		publishFeedback(action_lie_down_ptr_);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	publishFeedback(action_lie_down_ptr_);
	publishResult(action_lie_down_ptr_);

	std::cout << "\nactionLieDownCallback()" << "\t" << namespace_ << "\tFINISH\n" << std::endl;

}

// ------------------------------------------------------------------- //

void Connection::actionLieDownNameCallback(const actor_sim_action::LieDownNameGoalConstPtr &goal) {

	std::cout << "\nactionLieDownNameCallback()" << "\t" << namespace_ << "\n" << std::endl;

	if ( !actor_ptr_.lock()->lieDown(goal->object_name, goal->lying_height, goal->rotation) ) {
		std::cout << "LieDownName request cannot be processed" << std::endl;
		publishAbort(action_lie_down_name_ptr_);
		return;
	}

	while ( !actor_ptr_.lock()->getActionInfo().isTerminated() ) {
		publishFeedback(action_lie_down_name_ptr_);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	publishFeedback(action_lie_down_name_ptr_);
	publishResult(action_lie_down_name_ptr_);

	std::cout << "\nactionLieDownNameCallback()" << "\t" << namespace_ << "\tFINISH\n" << std::endl;

}

// ------------------------------------------------------------------- //

ignition::math::Pose3d Connection::transformPoint(const std::string &frame, const double &x_pos, const double &y_pos, const double &z_pos) {

	geometry_msgs::PoseStamped pose_global;

	// this is not necessary if point is defined in the actor global frame
	tf_listener_ptr_->transformPose(actor::FrameGlobal::getFrame(),
									actor::ros_interface::Conversion::convertIgnVectorToPoseStamped(ignition::math::Vector3d(x_pos, y_pos, z_pos), frame, false),
									pose_global);

	ignition::math::Pose3d pose;
	pose.Pos().X() = static_cast<double>(pose_global.pose.position.x);
	pose.Pos().Y() = static_cast<double>(pose_global.pose.position.y);
	pose.Pos().Z() = static_cast<double>(pose_global.pose.position.z);

	return (pose);

}

// ------------------------------------------------------------------- //

Connection::~Connection() {
	delete action_follow_object_ptr_;
	delete action_set_goal_ptr_;
	delete action_set_goal_name_ptr_;
	delete action_lie_down_ptr_;
	delete action_lie_down_name_ptr_;
}

// ------------------------------------------------------------------- //

/// \section FollowObjectAction
void Connection::publishAbort(actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>* as_ptr) {
	actor_sim_action::FollowObjectResult result;
	result.status = actor::core::Action::ABORTED;
	result.text = "'FollowObject' action request cannot be processed";
	as_ptr->setAborted(result, "action aborted");
}

void Connection::publishFeedback(actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>* as_ptr) {
	actor_sim_action::FollowObjectFeedback feedback;
	feedback.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	feedback.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->publishFeedback(feedback);
}
void Connection::publishResult(actionlib::SimpleActionServer<actor_sim_action::FollowObjectAction>* as_ptr) {
	actor_sim_action::FollowObjectResult result;
	result.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	result.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->setSucceeded(result, "finished");
}

// ------------------------------------------------------------------- //

/// \section SetGoal
void Connection::publishAbort(actionlib::SimpleActionServer<actor_sim_action::SetGoalAction>* as_ptr) {
	actor_sim_action::SetGoalResult result;
	result.status = actor::core::Action::ABORTED;
	result.text = "'SetGoal' action request cannot be processed";
	as_ptr->setAborted(result, "action aborted");
}
void Connection::publishFeedback(actionlib::SimpleActionServer<actor_sim_action::SetGoalAction>* as_ptr) {
	actor_sim_action::SetGoalFeedback feedback;
	feedback.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	feedback.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->publishFeedback(feedback);
}
void Connection::publishResult(actionlib::SimpleActionServer<actor_sim_action::SetGoalAction>* as_ptr) {
	actor_sim_action::SetGoalResult result;
	result.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	result.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->setSucceeded(result, "finished");
}

// ------------------------------------------------------------------- //

/// \section SetGoalName
void Connection::publishAbort(actionlib::SimpleActionServer<actor_sim_action::SetGoalNameAction>* as_ptr) {
	actor_sim_action::SetGoalNameResult result;
	result.status = actor::core::Action::ABORTED;
	result.text = "'SetGoalName' action request cannot be processed";
	as_ptr->setAborted(result, "action aborted");
}
void Connection::publishFeedback(actionlib::SimpleActionServer<actor_sim_action::SetGoalNameAction>* as_ptr) {
	actor_sim_action::SetGoalNameFeedback feedback;
	feedback.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	feedback.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->publishFeedback(feedback);
}
void Connection::publishResult(actionlib::SimpleActionServer<actor_sim_action::SetGoalNameAction>* as_ptr) {
	actor_sim_action::SetGoalNameResult result;
	result.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	result.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->setSucceeded(result, "finished");
}

// ------------------------------------------------------------------- //

/// \section LieDown
void Connection::publishAbort(actionlib::SimpleActionServer<actor_sim_action::LieDownAction>* as_ptr) {
	actor_sim_action::LieDownResult result;
	result.status = actor::core::Action::ABORTED;
	result.text = "'LieDown' action request cannot be processed";
	as_ptr->setAborted(result, "action aborted");
}
void Connection::publishFeedback(actionlib::SimpleActionServer<actor_sim_action::LieDownAction>* as_ptr) {
	actor_sim_action::LieDownFeedback feedback;
	feedback.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	feedback.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->publishFeedback(feedback);
}
void Connection::publishResult(actionlib::SimpleActionServer<actor_sim_action::LieDownAction>* as_ptr) {
	actor_sim_action::LieDownResult result;
	result.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	result.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->setSucceeded(result, "finished");
}

// ------------------------------------------------------------------- //

/// \section LieDownName
void Connection::publishAbort(actionlib::SimpleActionServer<actor_sim_action::LieDownNameAction>* as_ptr) {
	actor_sim_action::LieDownNameResult result;
	result.status = actor::core::Action::ABORTED;
	result.text = "'LieDownName' action request cannot be processed";
	as_ptr->setAborted(result, "action aborted");
}
void Connection::publishFeedback(actionlib::SimpleActionServer<actor_sim_action::LieDownNameAction>* as_ptr) {
	actor_sim_action::LieDownNameFeedback feedback;
	feedback.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	feedback.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->publishFeedback(feedback);
}
void Connection::publishResult(actionlib::SimpleActionServer<actor_sim_action::LieDownNameAction>* as_ptr) {
	actor_sim_action::LieDownNameResult result;
	result.status = static_cast<int>(actor_ptr_.lock()->getActionInfo().getStatus());
	result.text = actor_ptr_.lock()->getActionInfo().getStatusDescription();
	as_ptr->setSucceeded(result, "finished");
}

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
