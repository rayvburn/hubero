/*
 * Actor.cpp
 *
 *  Created on: Apr 9, 2019
 *      Author: rayvburn
 */

#include "core/Actor.h"
#include <sdf/Element.hh> 	// TODO: not used at the moment


namespace actor {
namespace core {

// ------------------------------------------------------------------- //

Actor::Actor():
		target_tolerance_(1.25), 		bounding_type_(ACTOR_BOUNDING_ELLIPSE),
		stance_(ACTOR_STANCE_STAND),	animation_factor_(4.50),
		trans_function_ptr(nullptr)
{

	//fsm_.addState(ACTOR_STATE_ALIGN_TARGET,  this->&stateHandlerAlignTarget(gazebo::common::UpdateInfo));
	// https://arduino.stackexchange.com/questions/14157/passing-class-member-function-as-argument
	//std::_Bind_helper<void (*)(const gazebo::common::UpdateInfo&)> abc = std::bind(Actor::stateHandlerAlignTarget, this);
//	fsm_.addState(ACTOR_STATE_ALIGN_TARGET,  std::bind(Actor::stateHandlerAlignTarget, this));
//	fsm_.addState(ACTOR_STATE_ALIGN_TARGET,  this->stateHandlerAlignTarget);
//	fsm_.addState(ACTOR_STATE_ALIGN_TARGET,  &stateHandlerAlignTarget);

	// compiles but throws "terminate called after throwing an instance of 'std::bad_function_call'
	//						what():  bad_function_call"
	// + #include <functional>
//	fsm_.addState(ACTOR_STATE_ALIGN_TARGET,  std::bind(&Actor::stateHandlerAlignTarget,   this, std::placeholders::_1));
//	fsm_.addState(ACTOR_STATE_MOVE_AROUND,   std::bind(&Actor::stateHandlerMoveAround, 	  this, std::placeholders::_1));
//	fsm_.addState(ACTOR_STATE_FOLLOW_OBJECT, std::bind(&Actor::stateHandlerFollowObject,  this, std::placeholders::_1));
//	fsm_.addState(ACTOR_STATE_TELEOPERATION, std::bind(&Actor::stateHandlerTeleoperation, this, std::placeholders::_1));



	std::cout << "ActorCLASS_CONSTRUCTOR FINISH - alignTarget address: " << &Actor::stateHandlerAlignTarget << std::endl;

}

// ------------------------------------------------------------------- //

void Actor::initGazeboInterface(const gazebo::physics::ActorPtr &actor, const gazebo::physics::WorldPtr &world) {

	// copy Gazebo world data pointers
	actor_ptr_ = actor;
	world_ptr_ = world;

	/* add an instance to common info class (missing velocity,
	 * bounding box of Actor in WorldPtr) */
	common_info_.addActor(actor_ptr_->GetName());

	// set initial stance and create custom trajectory
	setStance(ACTOR_STANCE_WALK);

	// add states for FSM
	fsm_.addState(ACTOR_STATE_ALIGN_TARGET);
	fsm_.addState(ACTOR_STATE_MOVE_AROUND);
	fsm_.addState(ACTOR_STATE_FOLLOW_OBJECT);
	fsm_.addState(ACTOR_STATE_TELEOPERATION);

	// update state handler function pointer
	if ( fsm_.didStateChange() ) {
		updateTransitionFunctionPtr();
	}

	// initialize SFM
	sfm_.Init(80.0, 2.0, 1.0, world_ptr_);

	/* set starting pose different to default to prevent actor
	 * to lie on his back 1 m above the ground*/
	ignition::math::Vector3d init_orient = actor_ptr_->WorldPose().Rot().Euler();
	ignition::math::Pose3d   init_pose;
	init_pose.Set(actor_ptr_->WorldPose().Pos(),
				  ignition::math::Quaterniond(init_orient.X() + IGN_PI/2,
											  init_orient.Y(),
											  init_orient.Z()));
	actor_ptr_->SetWorldPose(init_pose);



}

// ------------------------------------------------------------------- //

void Actor::setNewTarget(const ignition::math::Pose3d &pose) {
	target_ = pose.Pos();
}

// ------------------------------------------------------------------- //

void Actor::setNewTarget(const std::string &object_name) {
	gazebo::physics::ModelPtr model = world_ptr_->ModelByName(object_name);
	setNewTarget(model->WorldPose());
}

// ------------------------------------------------------------------- //

void Actor::followObject(const std::string &object_name) {
	object_to_follow_ = object_name;
	fsm_.setState(ACTOR_STATE_FOLLOW_OBJECT);
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

void Actor::setStance(const actor::ActorStance &stance_type) {

	stance_ = stance_type;
	std::string animation = convertStanceToAnimationName();
	gazebo::physics::Actor::SkeletonAnimation_M skeleton_anims = actor_ptr_->SkeletonAnimations();

	for (auto& x: skeleton_anims) {
		std::cout << "Skeleton animation: " <<  x.first << std::endl;
	}

	if ( skeleton_anims.find(animation) == skeleton_anims.end() ) {

		std::cout << "Skeleton animation " << animation << " not found.\n";

	} else {

		// Create custom trajectory
		trajectory_info_.reset(new gazebo::physics::TrajectoryInfo());
		trajectory_info_->type = animation;
		trajectory_info_->duration = 1.0;
		actor_ptr_->SetCustomTrajectory(trajectory_info_);

	}

}

// ------------------------------------------------------------------- //

void Actor::setState(const actor::ActorState &new_state) {
	fsm_.setState(new_state);
}

// ------------------------------------------------------------------- //

void Actor::executeTransitionFunction(const gazebo::common::UpdateInfo &info) {
//	gazebo::common::UpdateInfo cpy;
//	cpy = info;
//	fsm_.executeCurrentState(info);
	// FIXME
	(this->*trans_function_ptr)(info);
}

// ------------------------------------------------------------------- //

void Actor::stateHandlerAlignTarget	(const gazebo::common::UpdateInfo &info) {

	prepareForUpdate(info);

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// copy the actor's current rotation to local variable
	ignition::math::Vector3d new_rpy = pose_world_.Rot().Euler();

	/* if already aligned - switch to a certain state, otherwise proceed to the next
	 * rotation procedure */
	if ( alignToTargetDirection(&new_rpy) ) {
		std::cout << "\n\n\t\t\tALIGNED\n\n";
#ifndef SILENT_
		std::cout << "\n\n\t\t\tALIGNED\n\n";
#endif
		fsm_.setState(actor::ACTOR_STATE_MOVE_AROUND);
	}

	// update the local copy of the actor's pose (orientation only)
	// FIXME: that was changed - TO CHECK
	ignition::math::Quaterniond quat(new_rpy);
	pose_world_.Rot() = quat;

//	pose_actor.Set(pose_actor.Pos(), new_rpy);
//	SetActorPose(ignition::math::Pose3d(pose_world.Pos(), ignition::math::Quaterniond(new_rpy)));

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	/* forced close-to-zero distance traveled to avoid actor oscillations;
	 * of course 0.0 linear distance is traveled when pure rotation is performed */
	applyUpdate(info, 0.0007);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerMoveAround	(const gazebo::common::UpdateInfo &info) {

	// Social Force Model
	double dt = prepareForUpdate(info);

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	ignition::math::Vector3d sf = sfm_.GetSocialForce(world_ptr_,
													 actor_ptr_->GetName(),
													 pose_world_,
													 velocity_lin_,
													 target_,
													 common_info_);

//	if ( print_info ) {
//		std::cout << "\t TOTAL force: " << sf << std::endl;
//		std::cout << "\t lin_vels_vector: ";
//		for ( int i = 0; i < actor_common_info.getLinearVelocitiesVector().size(); i++ ) {
//			std::cout << "\t" << actor_common_info.getLinearVelocitiesVector()[i];
//		}
//		std::cout << std::endl;
//		std::cout << "***********************  NEW_POSE_CALC  **************************" << std::endl;
//	}

	ignition::math::Pose3d new_pose = sfm_.GetNewPose( pose_world_, pose_world_prev_, velocity_lin_, target_, sf, dt, 0);

//	if ( print_info ) {
//		std::cout << "\t NEW pose: " << new_pose;
//		std::cout << "\t\t distance to TARGET: " << (target - pose_actor.Pos()).Length() << std::endl;
//		std::cout << std::endl << std::endl;
//	}

	// calculate a distance to a target
	double to_target_distance = (target_ - pose_world_.Pos()).Length();

	// choose a new target position if the actor has reached its current target
//	if (to_target_distance < 0.3) {

	/* the smaller tolerance the bigger probability that actor will
	 * step into some obstacle */
	// TODO: YAML PARAMETER
	if (to_target_distance < target_tolerance_) {

		chooseNewTarget(info);
		// after setting new target, first let's rotate to its direction
		fsm_.setState(actor::ACTOR_STATE_ALIGN_TARGET);

	}

	// make sure the actor won't go out of bounds
	// TODO: YAML config
//	new_pose.Pos().X( std::max(-3.0,  std::min( 3.5, new_pose.Pos().X() ) ) );
//	new_pose.Pos().Y( std::max(-10.0, std::min( 2.0, new_pose.Pos().Y() ) ) );

	// object info update
	double dist_traveled = (new_pose.Pos() - actor_ptr_->WorldPose().Pos()).Length();

	// update the local copy of the actor's pose
//	SetActorPose(new_pose);
	pose_world_ = new_pose;

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	applyUpdate(info, dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerFollowObject	(const gazebo::common::UpdateInfo &info) {

	double dt = prepareForUpdate(info);
	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	double dist_traveled = 0.007;
	applyUpdate(info, dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerTeleoperation (const gazebo::common::UpdateInfo &info) {

	double dt = prepareForUpdate(info);
	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	double dist_traveled = 0.007;
	applyUpdate(info, dist_traveled);

}

// ------------------------------------------------------------------- //





void Actor::chooseNewTarget(const gazebo::common::UpdateInfo &info) {

	ignition::math::Vector3d new_target(target_);

	// look for target that is located at least 2 meters from current one
	while ((new_target - target_).Length() < 2.0) {

		// get random coordinates based on world limits
		// TODO: world limits loaded as a parameter
		new_target.X(ignition::math::Rand::DblUniform(-3, 3.5));
		new_target.Y(ignition::math::Rand::DblUniform(-10, 2));

		// check distance to all world's objects
		for (unsigned int i = 0; i < world_ptr_->ModelCount(); ++i) {

			/* distance-based target selection - could fail for very big objects
			 *
			//double dist = (world_ptr_->ModelByIndex(i)->WorldPose().Pos() - new_target).Length();
			if (dist < 2.0) {
				// if distance to some object is less than 2 meters
				// discard - discard current target and look
				// for another one
				new_target = target;
				break;
			}
			*
			*/

			/* bounding-box-based target selection - more safe for big obstacles,
			 * accounting some tolerance for a target accomplishment - an actor should
			 * not step into an object */

			// FIXME: cafe is a specific model that represents a whole world
			if ( world_ptr_->ModelByIndex(i)->GetName() == "cafe" ) {
				continue;
			}

			// check if model's bounding box contains target point
			if ( doesBoundingBoxContainPoint(world_ptr_->ModelByIndex(i)->BoundingBox(), target_) ) {
				// TODO: make this an error log message
				std::cout << "chooseNewTarget() - selection failed -> model containing target's pos: " << world_ptr_->ModelByIndex(i)->GetName() << std::endl;
				std::cout << std::endl;
				new_target = target_;
				break;
			}

		} // for

	} // while

	// finally found a new target
	target_ = new_target;

	// save event time
	time_last_target_selection_ = info.simTime;

}

// ------------------------------------------------------------------- //

bool Actor::isTargetStillReachable(const gazebo::common::UpdateInfo &info) {

	// check periodically, no need to do this in each iteration
	if ( (info.simTime - time_last_reachability_).Double() > 2.0 ) {

		// save event time
		time_last_reachability_ = info.simTime;

		// iterate over all models
		for (unsigned int i = 0; i < world_ptr_->ModelCount(); ++i) {

			// FIXME: cafe is a specific model that represents whole world
			if ( world_ptr_->ModelByIndex(i)->GetName() == "cafe" ) {
				continue;
			}

			// check if model's bounding box contains target point
			if ( doesBoundingBoxContainPoint(world_ptr_->ModelByIndex(i)->BoundingBox(), target_) ) {

				std::cout << "isTargetStillReachable()" << std::endl;
				std::cout << "\t" << actor_ptr_->GetName() << "\tDETECTED TARGET UNREACHABLE!" << std::endl;
				std::cout << "\ttarget: " << target_ << "\tmodel containing: " << world_ptr_->ModelByIndex(i)->GetName() << std::endl;
				std::cout << std::endl;
				std::cout << std::endl;
				std::cout << std::endl;
				return (false);

			}

		}

	}

	return (true);

}

// ------------------------------------------------------------------- //

bool Actor::isTargetNotReachedForTooLong(const gazebo::common::UpdateInfo &info) const {

	// TODO: make the time a YAML parameter
	if ( (info.simTime - time_last_target_selection_).Double() > 60.0 ) {

		std::cout << "isTargetNotReachedForTooLong()" << std::endl;
		std::cout << "\t" << actor_ptr_->GetName() << "\tDETECTED TARGET UNREACHABLE IN FINITE TIME!" << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		return (true);

	}
	return (false);

}

// ------------------------------------------------------------------- //

bool Actor::doesBoundingBoxContainPoint(const ignition::math::Box &bb, const ignition::math::Vector3d &pt) const {

	// check if model's bounding box is valid (not 0-length - for actors it is - || NaN || inf)
	if ( !std::isnan(bb.Max().Length()) && !std::isinf(bb.Max().Length()) && (bb.Max().Length() > 1e-06) ) {

		// check if model's bounding box contains target point
		if ( bb.Contains(pt) ) {
			return (true);
		}

	}
	return (false);

}

// ------------------------------------------------------------------- //

void Actor::updateStanceOrientation() {

	/* Corrects the rotation to align face with x axis if yaw = 0
	 * and stand up (with roll 0 actor is lying) */

	ignition::math::Vector3d rpy = pose_world_.Rot().Euler();

	switch (stance_) {

		// Yaw alignment with world X-axis DEPRECATED //
		case(actor::ACTOR_STANCE_WALK):
				rpy.X(1.5707);
				break;
		case(actor::ACTOR_STANCE_STAND):
				rpy.X(1.5707);
				break;
		case(actor::ACTOR_STANCE_LIE):
				rpy.X(0.0000);
				break;

	}

	pose_world_.Rot().Euler(rpy);

}

// ------------------------------------------------------------------- //

bool Actor::alignToTargetDirection(ignition::math::Vector3d *rpy) {

	//std::cout << "alignToTargetDirection()" << std::endl;

	// calculate the yaw angle actor need to rotate around world's Z axis
	// NOTE: +90 deg because actor's coordinate system is rotated 90 deg counter clockwise
	// V1 ------------------------------------------------------------------------------------------------------
//	ignition::math::Angle yaw_target(std::atan2( target.Y(), target.X() ) + (IGN_PI/2) );
	// V2 ------------------------------------------------------------------------------------------------------
	// yaw target expressed as an angle that depends on current IDEAL_TO_TARGET vector
	ignition::math::Vector3d to_target_vector = target_ - pose_world_.Pos();							// in world coord. system
	to_target_vector.Normalize();
	ignition::math::Angle yaw_target(std::atan2( to_target_vector.Y(), to_target_vector.X() ) + (IGN_PI/2) );	// +90 deg transforms the angle from actor's coord. system to the world's one
	//    ------------------------------------------------------------------------------------------------------
	yaw_target.Normalize();

	double yaw_start = pose_world_.Rot().Yaw();
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
	pose_world_ = actor_ptr_->WorldPose();

	updateStanceOrientation();

	double dt = (info.simTime - time_last_update_).Double();
	calculateVelocity(dt);

	common_info_.setLinearVel(velocity_lin_);

	// FIXME: delete - below just doesn't work - WorldPtr doesnt get updated
	actor_ptr_->SetLinearVel(velocity_lin_);

	/* update the bounding box/circle/ellipse of the actor
	 * (aim is to create a kind of an inflation layer) */
	switch(	bounding_type_ ) {

	case(ACTOR_BOUNDING_BOX):
			bounding_box_.updatePose(pose_world_);
			common_info_.setBoundingBox(bounding_box_);
			break;

	case(ACTOR_BOUNDING_CIRCLE):
			bounding_circle_.setCenter(pose_world_.Pos());
			common_info_.setBoundingCircle(bounding_circle_);
			break;

	case(ACTOR_BOUNDING_ELLIPSE):
			// correct yaw angle to make ellipse abstract from Actor coordinate system's orientation
			ignition::math::Angle yaw_world( pose_world_.Rot().Yaw() - IGN_PI_2);
			yaw_world.Normalize();
			bounding_ellipse_.updatePose(ignition::math::Pose3d(	pose_world_.Pos(),
																ignition::math::Quaterniond(pose_world_.Rot().Roll(),
																							pose_world_.Rot().Pitch(),
																							yaw_world.Radian()) ));
			common_info_.setBoundingEllipse(bounding_ellipse_);
			break;

	}

	// dt is helpful for further calculations
	return (dt);

}

// ------------------------------------------------------------------- //

void Actor::applyUpdate(const gazebo::common::UpdateInfo &info, const double &dist_traveled) {

  	// save last position to calculate velocity
	pose_world_prev_ = actor_ptr_->WorldPose();

	// update the global pose
	actor_ptr_->SetWorldPose(pose_world_, false, false);

	/*
	 * std::cout << actor->GetName() << " | script time: " << actor->ScriptTime() << "\tdist_trav: " << _dist_traveled << "\tanim_factor: " << animation_factor << std::endl;
	 *
	 * for some reason (very likely some very small number returned as interaction force
	 * in social force model) dist_traveled sometimes turns out to be a NaN - then change
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
	actor_ptr_->SetScriptTime( actor_ptr_->ScriptTime() + (dist_traveled * animation_factor_) );

	// update time
	time_last_update_ = info.simTime;

	// check if there has been some obstacle put into world since last target selection
	if ( !isTargetStillReachable(info) ) {
		chooseNewTarget(info);
		// after setting new target, first let's rotate to its direction
		// state will be changed in the next iteration
		fsm_.setState(actor::ACTOR_STATE_ALIGN_TARGET);
	}

	// check if actor is stuck
	if ( isTargetNotReachedForTooLong(info) ) {
		chooseNewTarget(info);
		// after setting new target, first let's rotate to its direction
		// state will be changed in the next iteration
		fsm_.setState(actor::ACTOR_STATE_ALIGN_TARGET);
	}

	// check whether state was updated
	if ( fsm_.didStateChange() ) {
		updateTransitionFunctionPtr();
	}

	// debug info
//	print_info = false;
//	Print_Set(false);

}

// ------------------------------------------------------------------- //

void Actor::calculateVelocity(const double &dt) {

	// =============== linear velocity

	ignition::math::Vector3d new_velocity;
	new_velocity.X( (pose_world_.Pos().X() - pose_world_prev_.Pos().X()) / dt );
	new_velocity.Y( (pose_world_.Pos().Y() - pose_world_prev_.Pos().Y()) / dt );
	new_velocity.Z( (pose_world_.Pos().Z() - pose_world_prev_.Pos().Z()) / dt );

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
	new_velocity.X( (pose_world_.Rot().Roll()  - pose_world_prev_.Rot().Roll()  ) / dt );
	new_velocity.Y( (pose_world_.Rot().Pitch() - pose_world_prev_.Rot().Pitch() ) / dt );
	new_velocity.Z( (pose_world_.Rot().Yaw()   - pose_world_prev_.Rot().Yaw()   ) / dt );

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

std::string Actor::convertStanceToAnimationName() {

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

Actor::~Actor() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
