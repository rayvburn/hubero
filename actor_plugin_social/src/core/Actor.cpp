/*
 * Actor.cpp
 *
 *  Created on: Apr 9, 2019
 *      Author: rayvburn
 */

#include "core/Actor.h"
#include <algorithm>    // std::find
#include <ignition/math/Line3.hh>
#include <memory> 		// make_unique


// FIXME: move to target manager later
#include <chrono>

// sleeper
#include <thread>

namespace actor {
namespace core {

// FIXME: init inflator private

// ------------------------------------------------------------------- //

Actor::Actor():
 		bounding_type_(ACTOR_BOUNDING_ELLIPSE),
		stance_(ACTOR_STANCE_STAND),
		trans_function_ptr(nullptr)
{

}

// ------------------------------------------------------------------- //

void Actor::initGazeboInterface(const gazebo::physics::ActorPtr &actor, const gazebo::physics::WorldPtr &world) {

	// copy Gazebo world data pointers
	actor_ptr_ = actor;
	world_ptr_ = world;

	/* add an instance to common info class (missing velocity,
	 * bounding box of Actor in WorldPtr) */
	common_info_.addActor(actor_ptr_->GetName());

}

// ------------------------------------------------------------------- //

void Actor::initRosInterface() {

	// run parameter loader
	params_ptr_ = std::make_shared<actor::ros_interface::ParamLoader>();
	params_ptr_->setActorParamsPrefix("actor");
	params_ptr_->setSfmParamsPrefix("sfm");
	params_ptr_->loadParameters(node_.getNodeHandlePtr());

	/* initialize SFM visualization instances */
	sfm_vis_single_.setColorLine(1.0f, 1.0f, 0.0f, 0.7f);
	sfm_vis_single_.setColorArrow(1.0f, 0.0f, 0.0f, 0.9f);

	if ( params_ptr_->getSfmVisParams().publish_grid ) {
		sfm_vis_grid_.setColorArrow(0.2f, 1.0f, 0.0f, 0.7f);
		/* make grid artificially bigger than a world's bounds;
		 * it makes random targets to be located further from
		 * walls */
		sfm_vis_grid_.createGrid(params_ptr_->getActorParams().world_bound_x.at(0) - 3.0, params_ptr_->getActorParams().world_bound_x.at(1) + 3.0,
								 params_ptr_->getActorParams().world_bound_y.at(0) - 3.0, params_ptr_->getActorParams().world_bound_y.at(1) + 3.0,
								 params_ptr_->getSfmVisParams().grid_resolution);
	}


	/* initialize ROS interface to allow publishing and receiving messages;
	 * due to inheritance from `enable_shared_from_this`, this method is created
	 * as a separate one */
	stream_.setNodeHandle(node_.getNodeHandlePtr());
	stream_.setNamespace(actor_ptr_->GetName());
	stream_.initPublisher<ActorMarkerType, visualization_msgs::Marker>(ActorMarkerType::ACTOR_MARKER_BOUNDING, "ellipse");
	stream_.initPublisher<ActorMarkerType, visualization_msgs::Marker>(ActorMarkerType::ACTOR_MARKER_SF_VECTOR, "social_force");
	stream_.initPublisher<ActorMarkerArrayType, visualization_msgs::MarkerArray>(ActorMarkerArrayType::ACTOR_MARKER_ARRAY_CLOSEST_POINTS, "closest_points");
	stream_.initPublisher<ActorNavMsgType, nav_msgs::Path>(ActorNavMsgType::ACTOR_NAV_PATH, "path");

	// check if grid usage has been enabled in .YAML file
	if ( params_ptr_->getSfmVisParams().publish_grid ) {
		stream_.initPublisher<ActorMarkerArrayType, visualization_msgs::MarkerArray>(ActorMarkerArrayType::ACTOR_MARKER_ARRAY_GRID, "force_grid");
	}

	// constructor of a Connection object
	connection_ptr_ = std::make_shared<actor::ros_interface::Connection>();

	// Connection's configuration
	connection_ptr_->setActorPtr( shared_from_this() ); // passes `std::shared_ptr` of `this` (which normally would be a const pointer)
	connection_ptr_->setNodeHandle(node_.getNodeHandlePtr());
	connection_ptr_->setNamespace(actor_ptr_->GetName());

	// initialize services for Actor control
	connection_ptr_->initServices();
	connection_ptr_->startCallbackProcessingThread();

	// initialize global plan provider
	//global_planner_.setNodeHandle(node_.getNodeHandlePtr());
	//global_planner_.setWaypointGap(10);

	// prev
//	global_planner_.initialize(node_.getNodeHandlePtr(), 10, actor_ptr_->GetName());
//	global_plan_ptr_ = std::shared_ptr<actor::ros_interface::GlobalPlan>(node_.getNodeHandlePtr(), 10, actor_ptr_->GetName());


}

// ------------------------------------------------------------------- //

void Actor::initInflator(const double &circle_radius) {

	// circle
	bounding_type_ = ACTOR_BOUNDING_CIRCLE;
	bounding_circle_.setRadius(circle_radius);
	bounding_circle_.setCenter(pose_world_ptr_->Pos());
	common_info_.setBoundingCircle(bounding_circle_);

}

// ------------------------------------------------------------------- //

void Actor::initInflator(const double &box_x_half, const double &box_y_half, const double &box_z_half) {

	// box
	bounding_type_ = ACTOR_BOUNDING_BOX;
	bounding_box_.init(box_x_half, box_y_half, box_z_half);
	bounding_box_.updatePose(*pose_world_ptr_);
	common_info_.setBoundingBox(bounding_box_);

}

// ------------------------------------------------------------------- //

void Actor::initInflator(const double &semi_major, const double &semi_minor, const double &center_offset_x,
						 const double &center_offset_y) {

	// ellipse
	bounding_type_ = ACTOR_BOUNDING_ELLIPSE;

	// correct yaw angle to make ellipse abstract from Actor coordinate system's orientation
	ignition::math::Angle yaw_world( pose_world_ptr_->Rot().Yaw() - IGN_PI_2);
	yaw_world.Normalize();
	bounding_ellipse_.init( semi_major, semi_minor, yaw_world.Radian(), pose_world_ptr_->Pos(), ignition::math::Vector3d(center_offset_x, center_offset_y, 0.0) );
	common_info_.setBoundingEllipse(bounding_ellipse_);

}

// ------------------------------------------------------------------- //

void Actor::initSFM() {

	/* choose a proper inflation type - convert from actor's inflation types
	 * available; number of types are not equal for actor and for SFM thus
	 * a conversion is done */
	sfm::core::InflationType sfm_inflation;

	/* using param value instead of member variable
	 * in case of different module initialization order */
	switch ( static_cast<actor::ActorBoundingType>(params_ptr_->getActorInflatorParams().bounding_type)  ) {

		case(actor::ActorBoundingType::ACTOR_BOUNDING_BOX):

			/* based on `box inflation type` parameter - choose the right
			 * SFM's box inflation type (actor is insensitive to this parameter);
			 * see sfm::core::InflationType */
			if ( static_cast<sfm::core::InflationType>(params_ptr_->getSfmParams().box_inflation_type) ==
					sfm::core::InflationType::INFLATION_BOX_ALL_OBJECTS ) {

				std::cout << "SFM init: inflation type - BOX ALL OBJECTS" << std::endl;
				sfm_inflation = sfm::core::InflationType::INFLATION_BOX_ALL_OBJECTS;

			} else if ( static_cast<sfm::core::InflationType>(params_ptr_->getSfmParams().box_inflation_type) ==
					sfm::core::InflationType::INFLATION_BOX_OTHER_OBJECTS ) {

				std::cout << "SFM init: inflation type - BOX OTHER OBJECTS" << std::endl;
				sfm_inflation = sfm::core::InflationType::INFLATION_BOX_OTHER_OBJECTS;

			}
			break;

		case(actor::ActorBoundingType::ACTOR_BOUNDING_CIRCLE):
				std::cout << "SFM init: inflation type - CIRCLE" << std::endl;
				sfm_inflation = sfm::core::InflationType::INFLATION_CIRCLE;
				break;

		case(actor::ActorBoundingType::ACTOR_BOUNDING_ELLIPSE):
				std::cout << "SFM init: inflation type - ELLIPSE" << std::endl;
				sfm_inflation = sfm::core::InflationType::INFLATION_ELLIPSE;
				break;

		case(actor::ActorBoundingType::ACTOR_NO_BOUNDING):
				/* TODO this bounding type seems to not have a correspondence in code? */
				std::cout << "SFM init: inflation type - NONE" << std::endl;
				sfm_inflation = sfm::core::InflationType::INFLATION_NONE;
				break;

	}

	// initialize SFM parameters
	sfm_.init(params_ptr_->getSfmParams().internal_force_factor,
			  params_ptr_->getSfmParams().interaction_force_factor,
			  static_cast<unsigned int>(params_ptr_->getSfmParams().mass),
			  params_ptr_->getSfmParams().max_speed,
			  params_ptr_->getSfmParams().fov,
			  params_ptr_->getSfmParams().min_force,
			  params_ptr_->getSfmParams().max_force,
			  static_cast<sfm::core::StaticObjectInteraction>(params_ptr_->getSfmParams().static_obj_interaction),
			  sfm_inflation,
			  world_ptr_);
	
}

// ------------------------------------------------------------------- //

void Actor::initActor(const sdf::ElementPtr sdf) {

	// - - - - - - - - - - - - - - - - - - - - - - -
	// finite state machine setup section
	// add states for FSM
	fsm_.addState(ACTOR_STATE_ALIGN_TARGET);
	fsm_.addState(ACTOR_STATE_MOVE_AROUND);
	fsm_.addState(ACTOR_STATE_FOLLOW_OBJECT);
	fsm_.addState(ACTOR_STATE_TELEOPERATION);

	// update state handler function pointer
	if ( fsm_.didStateChange() ) {
		updateTransitionFunctionPtr();
	}


	// - - - - - - - - - - - - - - - - - - - - - - -
	// target manager initialization section
	pose_world_ptr_ = std::make_shared<ignition::math::Pose3d>();
	target_manager_ = Target(world_ptr_, pose_world_ptr_, params_ptr_);
	target_manager_.initializeGlobalPlan(node_.getNodeHandlePtr(), 13, actor_ptr_->GetName());


	// - - - - - - - - - - - - - - - - - - - - - - -
	// initial stance setup section
	// set initial stance and create custom trajectory
	setStance( static_cast<actor::ActorStance>(params_ptr_->getActorParams().init_stance) );


	// - - - - - - - - - - - - - - - - - - - - - - -
	// initial pose setup section
	/* set starting pose different to default to prevent actor
	 * lying on his back 1 m above the ground */
	ignition::math::Pose3d init_pose;

	// check if some values have been set in .YAML
	if ( params_ptr_->getActorParams().init_pose.size() == 0 ) {

		// process parameters added to .world file (.sdf)
		ignition::math::Vector3d init_orient = actor_ptr_->WorldPose().Rot().Euler();
		init_pose.Set( actor_ptr_->WorldPose().Pos(), ignition::math::Quaterniond(init_orient.X(), init_orient.Y(), init_orient.Z()) );

	} else {

		// set pose as set in .YAML file - handy for single actor
		// not very handy for 100 of them
		init_pose.Set( params_ptr_->getActorParams().init_pose.at(0), params_ptr_->getActorParams().init_pose.at(1), params_ptr_->getActorParams().init_pose.at(2), params_ptr_->getActorParams().init_pose.at(3), params_ptr_->getActorParams().init_pose.at(4), params_ptr_->getActorParams().init_pose.at(5) );

	}
	updateStanceOrientation(init_pose);
	actor_ptr_->SetWorldPose(init_pose);

	// set previous pose to prevent velocity overshoot
	pose_world_prev_ = actor_ptr_->WorldPose();


	// - - - - - - - - - - - - - - - - - - - - - - -
	// bounding setup section
	// convert int to enum value and initialize a proper inflator/bounding
	bounding_type_ = static_cast<actor::ActorBoundingType>(params_ptr_->getActorInflatorParams().bounding_type);

	switch ( bounding_type_ ) {
	case(ACTOR_BOUNDING_BOX):
			initInflator( params_ptr_->getActorInflatorParams().box_size.at(0), params_ptr_->getActorInflatorParams().box_size.at(1), params_ptr_->getActorInflatorParams().box_size.at(2) );
			break;
	case(ACTOR_BOUNDING_CIRCLE):
			initInflator( params_ptr_->getActorInflatorParams().circle_radius );
			break;
	case(ACTOR_BOUNDING_ELLIPSE):
			initInflator( params_ptr_->getActorInflatorParams().ellipse.at(0), params_ptr_->getActorInflatorParams().ellipse.at(1), params_ptr_->getActorInflatorParams().ellipse.at(2), params_ptr_->getActorInflatorParams().ellipse.at(3) );
			break;
	}


	// - - - - - - - - - - - - - - - - - - - - - - -
	// initial target setup section

	// check if target coordinates have been set in .YAML - vector of 3 elements expected
	if ( params_ptr_->getActorParams().init_target.size() == 3 ) {

		// set target according to .YAML
		target_manager_.setNewTarget(ignition::math::Vector3d( params_ptr_->getActorParams().init_target.at(0),
															   params_ptr_->getActorParams().init_target.at(1),
															   params_ptr_->getActorParams().init_target.at(2) ));

	} else if ( sdf && sdf->HasElement("target") ) {

		// target coordinates in .YAML haven't been defined - use .sdf
		target_manager_.setNewTarget(sdf->Get<ignition::math::Vector3d>("target"));

	} else {

		/*
		// improper/no position set - choose random target during first OnUpdate TODO
		gazebo::common::UpdateInfo info_init;
		info_init.simTime = world_ptr_->SimTime();
		target_manager_.chooseNewTarget(info_init); // do not execute this (at least not now)
		*/

		// No target set but do not try to choose it now as costmap is very likely
		// not initialized yet. Proper state handlers will take care of that afterwards.

	}

	// - - - - - - - - - - - - - - - - - - - - - - -
	// initialize SFM with loaded parameters set
	initSFM();

	// - - - - - - - - - - - - - - - - - - - - - - -
	// debugging section
//	std::cout << actor_ptr_->GetName() << "\tIGNORED MODELS VECTOR:" << std::endl;
//	for ( size_t i = 0; i < params_ptr_->getSfmDictionary().ignored_models_.size(); i++ ) {
//		std::cout << i << "\t" << params_ptr_->getSfmDictionary().ignored_models_.at(i) << std::endl;
//	}
//	std::cout << "\n\n" << std::endl;

}

// ------------------------------------------------------------------- //

void Actor::readSDFParameters(const sdf::ElementPtr sdf) {

	// DEPRECATED?
//	if ( sdf && sdf->HasElement("target") ) {
//		this->target = sdf->Get<ignition::math::Vector3d>("target");
//	} else {
//		this->target = ignition::math::Vector3d(0, -5, 1.2138);
//	}
//
//	// Read in the target weight
//	if ( sdf->HasElement("target_weight") ) {
//		this->targetWeight = sdf->Get<double>("target_weight");
//	} else {
//		this->targetWeight = 1.15;
//	}
//
//	// Read in the obstacle weight
//	if ( sdf ->HasElement("obstacle_weight") ) {
//		this->obstacleWeight = sdf ->Get<double>("obstacle_weight");
//	} else {
//		this->obstacleWeight = 1.5;
//	}
//
//	// Read in the animation factor (applied in the OnUpdate function).
//	if ( sdf ->HasElement("animation_factor") ) {
//		this->animation_factor = sdf ->Get<double>("animation_factor");
//	} else {
//		this->animation_factor = 4.5;
//	}
//
//	// Add our own name to models we should ignore when avoiding obstacles.
//	this->ignoreModels.push_back(this->actor->GetName());
//
//	// Read in the other obstacles to ignore
//	if ( sdf ->HasElement("ignore_obstacles") ) {
//		sdf::ElementPtr modelElem =	sdf->GetElement("ignore_obstacles")->GetElement("model");
//		while (modelElem) {
//			this->ignoreModels.push_back(modelElem->Get<std::string>());
//			modelElem = modelElem->GetNextElement("model");
//		}
//	}

}

// ------------------------------------------------------------------- //

bool Actor::followObject(const std::string &object_name, const bool &stop_after_arrival) {

	// TODO: stop_after_arrival
	// FIXME: fill handler for this state
	object_to_follow_ = object_name;
	fsm_.setState(ACTOR_STATE_FOLLOW_OBJECT);

	return (true);

}

// ------------------------------------------------------------------- //

std::array<double, 3> Actor::getVelocity() const {

	std::array<double, 3> array;
	array.at(0) = velocity_lin_.X();
	array.at(1) = velocity_lin_.Y();
	array.at(2) = velocity_ang_.Z();
	return (array);

}

// ------------------------------------------------------------------- //

bool Actor::setStance(const actor::ActorStance &stance_type) {

	if ( stance_ != stance_type ) {

		stance_ = stance_type;
		std::string animation = convertStanceToAnimationName();
		gazebo::physics::Actor::SkeletonAnimation_M skeleton_anims = actor_ptr_->SkeletonAnimations();

		/* To print available animations:
		for (auto& x: skeleton_anims) { std::cout << "Skel. animation: " << x.first << std::endl; }
		*/

		if ( skeleton_anims.find(animation) == skeleton_anims.end() ) {

			std::cout << "Skeleton animation " << animation << " not found.\n";
			return (false);

		} else {

			// Create custom trajectory
			trajectory_info_.reset(new gazebo::physics::TrajectoryInfo());
			trajectory_info_->type = animation;
			trajectory_info_->duration = 1.0;
			actor_ptr_->SetCustomTrajectory(trajectory_info_);
			return (true);

		}

	}
	return (false);

}

// ------------------------------------------------------------------- //

bool Actor::setState(const actor::ActorState &new_state) {

	// check if state is valid
	unsigned int state_to_be = static_cast<unsigned int>(new_state);
	unsigned int state_lower_bound = static_cast<unsigned int>(actor::ACTOR_STATE_ALIGN_TARGET);

	// TODO:
	// NOTE: below needs to be adjusted if new state will be added!
	unsigned int state_upper_bound = static_cast<unsigned int>(actor::ACTOR_STATE_TELEOPERATION);

	if ( (state_to_be >= state_lower_bound) && (state_to_be <= state_upper_bound) ) {
		fsm_.setState(new_state);
		return (true);
	} else {
		return (false);
	}

}

// ------------------------------------------------------------------- //

void Actor::executeTransitionFunction(const gazebo::common::UpdateInfo &info) {
	(this->*trans_function_ptr)(info);
}

// ------------------------------------------------------------------- //

void Actor::stateHandlerAlignTarget	(const gazebo::common::UpdateInfo &info) {

	// ---------------------------------------------

	prepareForUpdate(info);

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// copy the actor's current rotation to local variable
	ignition::math::Vector3d new_rpy = pose_world_ptr_->Rot().Euler();

	/* if already aligned - switch to a certain state, otherwise proceed to the next
	 * rotation procedure */
	if ( alignToTargetDirection(&new_rpy) ) {
#ifndef SILENT_
		std::cout << "\n\n\t\t\tALIGNED\n\n";
#endif
		fsm_.setState(actor::ACTOR_STATE_MOVE_AROUND);
	}

	// update the local copy of the actor's pose (orientation only)
	ignition::math::Quaterniond quat(new_rpy);
	pose_world_ptr_->Rot() = quat;

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	/* forced close-to-zero distance traveled to avoid actor oscillations;
	 * of course 0.0 linear distance is traveled when pure rotation is performed */
	applyUpdate(info, params_ptr_->getActorParams().animation_speed_rotation);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerMoveAround	(const gazebo::common::UpdateInfo &info) {

	// ---------------------------------------------

	// Social Force Model
	double dt = prepareForUpdate(info);

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// check whether a target exists
	if ( !target_manager_.isTargetChosen() ) {
		if ( !target_manager_.chooseNewTarget(info) ) {
			return;
		}
	}

	// check whether a target has plan generated
	if ( !target_manager_.isPlanGenerated() ) {
		if ( !target_manager_.generatePathPlan(target_manager_.getTarget()) ) {
			target_manager_.abandonTarget();
		}
	}

	ignition::math::Vector3d sf = sfm_.computeSocialForce(world_ptr_, actor_ptr_->GetName(), *pose_world_ptr_,
														  velocity_lin_, target_manager_.getCheckpoint(),
														  common_info_, dt); // target_

//	if ( print_info ) {
//		std::cout << "\t TOTAL force: " << sf << std::endl;
//		std::cout << "\t lin_vels_vector: ";
//		for ( int i = 0; i < actor_common_info.getLinearVelocitiesVector().size(); i++ ) {
//			std::cout << "\t" << actor_common_info.getLinearVelocitiesVector()[i];
//		}
//		std::cout << std::endl;
//		std::cout << "***********************  NEW_POSE_CALC  **************************" << std::endl;
//	}

	ignition::math::Pose3d new_pose = sfm_.computeNewPose(*pose_world_ptr_, velocity_lin_, sf, dt);

//	if ( print_info ) {
//		std::cout << "\t NEW pose: " << new_pose;
//		std::cout << "\t\t distance to TARGET: " << (target - pose_actor.Pos()).Length() << std::endl;
//		std::cout << std::endl << std::endl;
//	}

	if ( target_manager_.isTargetReached() ) {

		target_manager_.chooseNewTarget(info);
		// after setting new target, first let's rotate to its direction
		fsm_.setState(actor::ACTOR_STATE_ALIGN_TARGET);

	} else if ( target_manager_.isCheckpointReached() ) {

		// take next checkpoint from vector (path)
		target_manager_.updateCheckpoint();

	}

	// object info update
	double dist_traveled = (new_pose.Pos() - actor_ptr_->WorldPose().Pos()).Length();

	// update the local copy of the actor's pose
	*pose_world_ptr_ = new_pose;

	// publish social force vector and closest points lines
	stream_.publishData(ActorMarkerType::ACTOR_MARKER_SF_VECTOR, sfm_vis_single_.createArrow(new_pose.Pos(), sf) );
	stream_.publishData(ActorMarkerArrayType::ACTOR_MARKER_ARRAY_CLOSEST_POINTS, sfm_vis_single_.createLineListArray(sfm_.getClosestPointsVector()) );

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	applyUpdate(info, dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerFollowObject	(const gazebo::common::UpdateInfo &info) {

	double dt = prepareForUpdate(info);
	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	double dist_traveled = 0.007; // temp
	applyUpdate(info, dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerTeleoperation (const gazebo::common::UpdateInfo &info) {

	double dt = prepareForUpdate(info);
	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	double dist_traveled = 0.007; // temp
	applyUpdate(info, dist_traveled);

}

// ------------------------------------------------------------------- //


// private section


// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

void Actor::updateStanceOrientation(ignition::math::Pose3d &pose) {

	/* Corrects the rotation to align face with x axis if yaw = 0
	 * and stand up (with roll 0 actor is lying) */

	ignition::math::Vector3d rpy = pose.Rot().Euler();

	switch (stance_) {

		case(actor::ACTOR_STANCE_WALK):
		case(actor::ACTOR_STANCE_STAND):
		case(actor::ACTOR_STANCE_STAND_UP):
		case(actor::ACTOR_STANCE_TALK_A):
		case(actor::ACTOR_STANCE_TALK_B):
		case(actor::ACTOR_STANCE_RUN):
		case(actor::ACTOR_STANCE_SIT_DOWN):
		case(actor::ACTOR_STANCE_SITTING):
				rpy.X(IGN_PI_2);
				break;

		case(actor::ACTOR_STANCE_LIE):
				rpy.X(0.0000);
				break;

	}

	pose.Rot().Euler(rpy);

}

// ------------------------------------------------------------------- //

bool Actor::alignToTargetDirection(ignition::math::Vector3d *rpy) {

	// calculate the yaw angle actor need to rotate around world's Z axis
	// NOTE: +90 deg because actor's coordinate system is rotated 90 deg counter clockwise
	// V1 ------------------------------------------------------------------------------------------------------
//	ignition::math::Angle yaw_target(std::atan2( target.Y(), target.X() ) + (IGN_PI/2) );
	// V2 ------------------------------------------------------------------------------------------------------
	// yaw target expressed as an angle that depends on current IDEAL_TO_TARGET vector
//	ignition::math::Vector3d to_target_vector = target_ - pose_world_ptr_->Pos();							// in world coord. system
	// NOTE: the above was before (before target_checkpoint_ version)
	ignition::math::Vector3d to_target_vector = target_manager_.getCheckpoint() - pose_world_ptr_->Pos();

	to_target_vector.Normalize();
	ignition::math::Angle yaw_target(std::atan2( to_target_vector.Y(), to_target_vector.X() ) + (IGN_PI/2) );	// +90 deg transforms the angle from actor's coord. system to the world's one
	//    ------------------------------------------------------------------------------------------------------
	yaw_target.Normalize();

	double yaw_start = pose_world_ptr_->Rot().Yaw();
	//ignition::math::Angle yaw_diff( yaw_start - yaw_target.Radian() );
	ignition::math::Angle yaw_diff( yaw_target.Radian() - yaw_start );
	yaw_diff.Normalize();

	/*
	// choose the right rotation direction (but the direction already calculated properly)
	if ( std::fabs(yaw_diff.Radian()) > (IGN_PI/2) ) {

		if ( yaw_diff.Radian() >= 1e-06 ) {
			// positive value
			yaw_diff.Radian( -(IGN_PI - yaw_diff.Radian() ));
		} else {
			// negative value
			yaw_diff.Radian( IGN_PI - std::fabs(yaw_diff.Radian() ));
		}

		if ( actor->GetName() == "actor1" ) {
			//if ( ctr == 0 ) {
				std::cout << "\tyaw_diff_changed: " << yaw_diff.Radian() << std::endl;
			//}
		}

	}
	*/

	// smooth the rotation if too big
	static const double YAW_INCREMENT = 0.001;
#define IVERT_SIGN // ok with that setting

#ifndef IVERT_SIGN
	short int sign = -1;
#else
	short int sign = +1;
#endif
//	ignition::math::Vector3d rpy = pose_actor.Rot().Euler();

	// check the sign of the diff - the movement should be performed in the OPPOSITE direction to yaw_diff angle
	if ( yaw_diff.Radian() < 0.0f ) {
#ifndef IVERT_SIGN
		sign = +1;
#else
		sign = -1;
#endif
	}

	// save the change to tell if actor is already aligned or not
	double angle_change = 0.0;

	// consider the difference (increment or decrement)
	if ( std::fabs(yaw_diff.Radian()) < YAW_INCREMENT ) {
		angle_change = static_cast<double>(sign) * yaw_diff.Radian();
	} else {
		angle_change = static_cast<double>(sign) * YAW_INCREMENT;
	}

	ignition::math::Angle yaw_result(yaw_start + angle_change);
	yaw_result.Normalize();
	rpy->Z(yaw_result.Radian());

	// return true if the yaw_diff is small enough, otherwise return false
	yaw_diff.Radian(yaw_diff.Radian() - angle_change);
	yaw_diff.Normalize();

	if ( std::fabs(yaw_diff.Radian()) < IGN_DTOR(10) ) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

double Actor::prepareForUpdate(const gazebo::common::UpdateInfo &info) {

	// copy pose
	*pose_world_ptr_ = actor_ptr_->WorldPose();

	updateStanceOrientation(*pose_world_ptr_); // FIXME

	double dt = (info.simTime - time_last_update_).Double();
	calculateVelocity(dt);

	common_info_.setLinearVel(velocity_lin_);

	// DELETE - below just doesn't work - WorldPtr doesnt get updated
	// actor_ptr_->SetLinearVel(velocity_lin_);

	// update bounding model's pose
	updateBounding(*pose_world_ptr_);

	// dt is helpful for further calculations
	return (dt);

}

// ------------------------------------------------------------------- //

void Actor::applyUpdate(const gazebo::common::UpdateInfo &info, const double &dist_traveled) {

  	// save last position to calculate velocity
	pose_world_prev_ = actor_ptr_->WorldPose();

	// make sure the actor won't go out of bounds
	if ( params_ptr_->getActorParams().limit_actors_workspace ) {
		pose_world_ptr_->Pos().X(std::max( params_ptr_->getActorParams().world_bound_x.at(0), std::min( params_ptr_->getActorParams().world_bound_x.at(1), pose_world_ptr_->Pos().X())));
		pose_world_ptr_->Pos().Y(std::max( params_ptr_->getActorParams().world_bound_y.at(0), std::min( params_ptr_->getActorParams().world_bound_y.at(1), pose_world_ptr_->Pos().Y())));
	}

	// update the global pose
	actor_ptr_->SetWorldPose(*pose_world_ptr_, false, false);


	/*
	 * std::cout << actor->GetName() << " | script time: " << actor->ScriptTime() << "\tdist_trav: " << _dist_traveled << "\tanim_factor: " << animation_factor << std::endl;
	 *
	 * for some reason (very likely some very small number returned as interaction force
	 * in social force model) dist_traveled sometimes turns out to be a NaN - one should then change
	 * it to a typical value of 0.005; NaN value of dist seems to be the cause of such error:
	 *
	 * "gazebo::common::NodeAnimation::FrameAt(double, bool) const: Assertion
	 *  `(t >= 0.0 && t <= 1.0)&&("t is not in the range 0.0..1.0")' failed."
	 *
	 *  FIXME: if such error occurs uncomment below line (not done for debugging process)
	 *  (std::isnan(_dist_traveled)) ? (_dist_traveled = 0.0005) : (0);
	 *
	 */


	// update script time to set proper animation speed
	actor_ptr_->SetScriptTime( actor_ptr_->ScriptTime() + (dist_traveled * params_ptr_->getActorParams().animation_factor) );

	// update time
	time_last_update_ = info.simTime;

	// check if there has been some obstacle put into world since last target selection
	if ( !target_manager_.isTargetStillReachable(info) ) {
		if ( target_manager_.chooseNewTarget(info) ) {
			// after setting new target, first let's rotate to its direction
			// state will be changed in the next iteration
			fsm_.setState(actor::ACTOR_STATE_ALIGN_TARGET);
		}
	}

	// check if actor is stuck
	if ( target_manager_.isTargetNotReachedForTooLong(info) ) {
		if ( target_manager_.chooseNewTarget(info) ) {
			// after setting new target, first let's rotate to its direction
			// state will be changed in the next iteration
			fsm_.setState(actor::ACTOR_STATE_ALIGN_TARGET);
		}
	}

	// check whether state was updated
	if ( fsm_.didStateChange() ) {
		updateTransitionFunctionPtr();
	}

	// TODO: vis freq as parameter

	// publish data for visualization
	// proper bounding object to publish needs to be chosen
	switch ( bounding_type_ ) {
	case(ACTOR_BOUNDING_BOX):
			stream_.publishData(ActorMarkerType::ACTOR_MARKER_BOUNDING, bounding_box_.getMarkerConversion());
			break;
	case(ACTOR_BOUNDING_CIRCLE):
			stream_.publishData(ActorMarkerType::ACTOR_MARKER_BOUNDING, bounding_circle_.getMarkerConversion());
			break;
	case(ACTOR_BOUNDING_ELLIPSE):
			stream_.publishData(ActorMarkerType::ACTOR_MARKER_BOUNDING, bounding_ellipse_.getMarkerConversion());
			break;
	}

	stream_.publishData(ActorTfType::ACTOR_TF_SELF, *pose_world_ptr_);
	stream_.publishData(ActorTfType::ACTOR_TF_TARGET, ignition::math::Pose3d(ignition::math::Vector3d(target_manager_.getTarget()),
																			 ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)));
	stream_.publishData(ActorTfType::ACTOR_TF_CHECKPOINT, ignition::math::Pose3d(ignition::math::Vector3d(target_manager_.getCheckpoint()),
																			 ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)));
	stream_.publishData(ActorNavMsgType::ACTOR_NAV_PATH, target_manager_.getPath());

	// check if grid publication has been enabled in parameter file
	if ( params_ptr_->getSfmVisParams().publish_grid ) {

		/* visualize SFM grid if enough time elapsed from last
		 * publication and there is some unit subscribing to a topic */
		if ( visualizeVectorField(info) ) {
			stream_.publishData( actor::ActorMarkerArrayType::ACTOR_MARKER_ARRAY_GRID, sfm_vis_grid_.getMarkerArray() );
		}

	}


	// ellipse
//	std::cout << "\tBOUNDING ELLIPSE\n";
//	std::cout << "\tActor's pos: " << pose_world_ptr_->Pos() << std::endl;
//	std::cout << "\tEllipse's offset: " << bounding_ellipse_.getCenterOffset() << std::endl;
//	std::cout << "\tEllipse's center: " << bounding_ellipse_.getCenter() << "\tEllipse's shifted center: " << bounding_ellipse_.getCenterShifted() << std::endl;
//	std::cout << "\n\n";

	// debug info
//	print_info = false;
//	Print_Set(false);

}

// ------------------------------------------------------------------- //

void Actor::updateBounding(const ignition::math::Pose3d &pose) {

	/* update the bounding box/circle/ellipse of the actor
	 * (aim is to create a kind of an inflation layer) */
	switch ( bounding_type_ ) {

	case(ACTOR_BOUNDING_BOX):
			bounding_box_.updatePose(pose);
			common_info_.setBoundingBox(bounding_box_);
			break;

	case(ACTOR_BOUNDING_CIRCLE):
			bounding_circle_.setCenter(pose.Pos());
			common_info_.setBoundingCircle(bounding_circle_);
			break;

	case(ACTOR_BOUNDING_ELLIPSE):
			// correct yaw angle to make ellipse abstract from Actor coordinate system's orientation
			ignition::math::Angle yaw_world( pose.Rot().Yaw() - IGN_PI_2);
			yaw_world.Normalize();
			bounding_ellipse_.updatePose(ignition::math::Pose3d( pose.Pos(),
																ignition::math::Quaterniond(pose.Rot().Roll(),
																							pose.Rot().Pitch(),
																							yaw_world.Radian()) ));
			common_info_.setBoundingEllipse(bounding_ellipse_);
			break;

	}

}

// ------------------------------------------------------------------- //

void Actor::calculateVelocity(const double &dt) {

	// =============== linear velocity

	ignition::math::Vector3d new_velocity;
	new_velocity.X( (pose_world_ptr_->Pos().X() - pose_world_prev_.Pos().X()) / dt );
	new_velocity.Y( (pose_world_ptr_->Pos().Y() - pose_world_prev_.Pos().Y()) / dt );
	new_velocity.Z( (pose_world_ptr_->Pos().Z() - pose_world_prev_.Pos().Z()) / dt );

	/* Velocity Averaging Block -
	 * used there to prevent some kind of oscillations in
	 * social force algorithm execution - the main reason of such behavior were changes
	 * in speed which cause relative velocity fluctuations which on turn affects final
	 * result a lot */
	std::rotate( velocities_lin_to_avg_.begin(), velocities_lin_to_avg_.begin()+1, velocities_lin_to_avg_.end() );
	velocities_lin_to_avg_.at(velocities_lin_to_avg_.size()-1) = new_velocity;

	// sum up - at the moment no Z-velocities are taken into consideration
	double vel[3] = {0.0, 0.0, 0.0};
	for ( size_t i = 0; i < velocities_lin_to_avg_.size(); i++ ) {
		vel[0] += velocities_lin_to_avg_.at(i).X();
		vel[1] += velocities_lin_to_avg_.at(i).Y();
		// vel[2] += velocities_to_avg.at(i).Z();
	}

	vel[0] = vel[0] / static_cast<double>(velocities_lin_to_avg_.size());
	vel[1] = vel[1] / static_cast<double>(velocities_lin_to_avg_.size());
	// vel[2] = vel[2] / static_cast<double>(velocities_to_avg.size()); // always 0

	new_velocity.X(vel[0]);
	new_velocity.Y(vel[1]);
	new_velocity.Z(vel[2]);

	velocity_lin_ = new_velocity;

	// =============== angular velocity
	new_velocity.X( (pose_world_ptr_->Rot().Roll()  - pose_world_prev_.Rot().Roll()  ) / dt );
	new_velocity.Y( (pose_world_ptr_->Rot().Pitch() - pose_world_prev_.Rot().Pitch() ) / dt );
	new_velocity.Z( (pose_world_ptr_->Rot().Yaw()   - pose_world_prev_.Rot().Yaw()   ) / dt );

	// averaging block is not needed as angular velocity is not used by SFM
	velocity_ang_.X( new_velocity.X() );
	velocity_ang_.Y( new_velocity.Y() );
	velocity_ang_.Z( new_velocity.Z() );

}

// ------------------------------------------------------------------- //

void Actor::updateTransitionFunctionPtr() {

	switch( fsm_.getState() ) {

	case(ACTOR_STATE_ALIGN_TARGET):
			std::cout << "State: \talignToTarget" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerAlignTarget; 		// &this->Actor::stateHandlerAlignTarget;
			setStance(ACTOR_STANCE_WALK);
			break;
	case(ACTOR_STATE_STUCK):
			std::cout << "State: \tgotStuck" << std::endl;
			// empty
			break;
	case(ACTOR_STATE_MOVE_AROUND):
			std::cout << "State: \tmoveAround" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerMoveAround; 		// &this->Actor::stateHandlerMoveAround;
			setStance(ACTOR_STANCE_WALK);
			break;
	case(ACTOR_STATE_STOP_AND_STARE):
			std::cout << "State: \tstopAndStare" << std::endl;
			// empty
			break;
	case(ACTOR_STATE_FOLLOW_OBJECT):
			std::cout << "State: \tfollowObject" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerFollowObject; 	// &this->Actor::stateHandlerFollowObject;
			setStance(ACTOR_STANCE_SITTING);
			break;
	case(ACTOR_STATE_TELEOPERATION):
			std::cout << "State: \tteleoperation" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerTeleoperation;
			setStance(ACTOR_STANCE_STAND);
			break;
	default:
			std::cout << "State: \tUNKNOWN" << std::endl;
			break;

	}

}

// ------------------------------------------------------------------- //

std::string Actor::convertStanceToAnimationName() const {

	std::string anim_name;

	switch( stance_ ) {

	case(ACTOR_STANCE_WALK):
			std::cout << "Stance: \tWALK" << std::endl;
			anim_name = "walk";
			break;
	case(ACTOR_STANCE_STAND):
			std::cout << "Stance: \tSTAND" << std::endl;
			anim_name = "stand";
			break;
	case(ACTOR_STANCE_LIE):
			std::cout << "Stance: \tLIE" << std::endl;
			anim_name = "stand"; // lie is stand but in different plane
			break;
	case(ACTOR_STANCE_SIT_DOWN):
			std::cout << "Stance: \tSIT_DOWN" << std::endl;
			anim_name = "sit_down";
			break;
	case(ACTOR_STANCE_SITTING):
			std::cout << "Stance: \tSITTING" << std::endl;
			anim_name = "sitting";
			break;
	case(ACTOR_STANCE_STAND_UP):
			std::cout << "Stance: \tSTAND_UP" << std::endl;
			anim_name = "stand_up";
			break;
	case(ACTOR_STANCE_RUN):
			std::cout << "Stance: \tRUN" << std::endl;
			anim_name = "run";
			break;
	case(ACTOR_STANCE_TALK_A):
			std::cout << "Stance: \tTALK_A" << std::endl;
			anim_name = "talk_a";
			break;
	case(ACTOR_STANCE_TALK_B):
			std::cout << "Stance: \tTALK_B" << std::endl;
			anim_name = "talk_b";
			break;
	default:
			std::cout << "Stance: \tUNKNOWN" << std::endl;
			break;

	}

	return (anim_name);

}


// ------------------------------------------------------------------- //

bool Actor::visualizeVectorField(const gazebo::common::UpdateInfo &info) {

	// do not publish too often
	if ( (info.simTime - time_last_vis_pub_).Double() > params_ptr_->getSfmVisParams().grid_pub_period ) {

		/* update the sim time even when grid will not be published
		 * to avoid calling getSubscribersNum() in each iteration */
		time_last_vis_pub_ = info.simTime;

		/* creating a grid with high resolution is pretty time-consuming
		 * check if there is a subscriber and then calculate force vectors
		 * for a whole grid */

		/* grid generation is orientation-dependent (current orientation
		 * of an actor is used) */
		if ( stream_.getSubscribersNum(actor::ActorMarkerArrayType::ACTOR_MARKER_ARRAY_GRID ) ) {

			ignition::math::Pose3d pose;	// pose where `virtual` actor will be placed in
			ignition::math::Vector3d sf;	// social force vector

			// before a start reset a grid index
			sfm_vis_grid_.resetGridIndex();

			while ( !sfm_vis_grid_.isWholeGridChecked() ) {

				// set an actor's virtual pose
				pose = ignition::math::Pose3d( sfm_vis_grid_.getNextGridElement(), pose_world_ptr_->Rot() );

				// update the bounding of an actor
				updateBounding(pose);

				// calculate social force for actor located in current pose
				// hard-coded time delta
//				sf = sfm_.computeSocialForce(world_ptr_, actor_ptr_->GetName(), pose, velocity_lin_, target_, common_info_, 0.001);
				sf = sfm_.computeSocialForce(world_ptr_, actor_ptr_->GetName(), pose, velocity_lin_, target_manager_.getCheckpoint(), common_info_, 0.001);

				// pass a result to vector of grid forces
				sfm_vis_grid_.addMarker( sfm_vis_grid_.createArrow(pose.Pos(), sf) );

			}
			return (true);

		} /* getSubscribersNum() */

	} /* if ( time_elapsed ) */

	return (false);

}

// ------------------------------------------------------------------- //

Actor::~Actor() { }

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
