/*
 * Target.cpp
 *
 *  Created on: Jul 4, 2019
 *      Author: rayvburn
 */

#include <core/Target.h>
#include <ignition/math/Rand.hh> // DblUniform
#include <core/Enums.h> // actor model id

namespace actor {
namespace core {

// ------------------------------------------------------------------- //
Target::Target(): has_target_(false), has_global_plan_(false), is_following_(false), followed_model_ptr_(nullptr) { }
// ------------------------------------------------------------------- //

Target::Target(gazebo::physics::WorldPtr world_ptr, std::shared_ptr<const ignition::math::Pose3d> pose_world_ptr,
			   std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr)
					: has_target_(false), has_global_plan_(false),
					  is_following_(false), followed_model_ptr_(nullptr) {

	world_ptr_ = world_ptr;
	pose_world_ptr_ = pose_world_ptr;
	params_ptr_ = params_ptr;

}

// ------------------------------------------------------------------- //

Target::Target(const Target &obj) {

	// copy ctor
	has_target_ = obj.has_target_;
	has_global_plan_ = obj.has_global_plan_;
	is_following_ = obj.is_following_;

	pose_world_ptr_ = obj.pose_world_ptr_;
	params_ptr_ = obj.params_ptr_;
	world_ptr_ = obj.world_ptr_;

}

// ------------------------------------------------------------------- //

void Target::initializeGlobalPlan(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id) {
	global_planner_.initialize(nh_ptr, gap, frame_id);
}

// ------------------------------------------------------------------- //

bool Target::followObject(const std::string &object_name) {

	// temp model ptr in case of a situation when 1 object is already followed
	// but tracked object name change has been commanded. This prevents the `old object` deletion.
	gazebo::physics::ModelPtr model_ptr_temp = nullptr;
	bool is_valid = false;

	// evaluate validity of an object
	std::tie(is_valid, model_ptr_temp) = isModelValid(object_name);
	if ( !is_valid ) {
		return (false);
	}

	// check distance between actor and an object (to-be-tracked)
	ignition::math::Vector3d dist = pose_world_ptr_->Pos() - model_ptr_temp->WorldPose().Pos();
	if ( dist.Length() <= params_ptr_->getActorParams().target_tolerance ) {
		std::cout << "Target::followObject - followed object is too close to actor to be tracked" << std::endl;
		return (false);
	}

	bool ok = false;
	ignition::math::Vector3d pt_intersection; // dynamic obstacles will not be marked in the costmap
	ignition::math::Vector3d line_dir;

	std::tie(ok, pt_intersection, std::ignore) = findBoxPoint(model_ptr_temp);
	if ( !ok ) {
		return (false);
	}

	// generate path plan and possibly update the internal state
	if ( tryToApplyTarget(pt_intersection) ) {
		// assign new model to the tracked object pointer
		followed_model_ptr_ = model_ptr_temp;
		is_following_ = true;
		return (true);
	}

	return (false);

}

// ------------------------------------------------------------------- //

bool Target::isFollowing() const {
	return (is_following_);
}

// ------------------------------------------------------------------- //

bool Target::updateFollowedTarget() {

	// check model's pointer validity (this is an operation which does not consume
	// much resources so can be performed in each iteration (opposite to a global path generation))
	if ( followed_model_ptr_ == nullptr ) {
		is_following_ = false;
		has_target_ = false;
		has_global_plan_ = false;
		return (false);
	}

	// periodically update the tracked object's position
	if ( (world_ptr_->SimTime() - time_last_follow_plan_).Double() >= 2.0 ) {

		// update event time
		time_last_follow_plan_ = world_ptr_->SimTime();

		// check distance to the model
		ignition::math::Vector3d dist = pose_world_ptr_->Pos() - followed_model_ptr_->WorldPose().Pos();
		if ( dist.Length() <= params_ptr_->getActorParams().target_tolerance ) {
			// return true as tracking proceeds well - the tracked
			// object has just stopped or moves very slowly
			return (true);
		}

		// try to find line-box intersection point
		bool found = false;
		ignition::math::Vector3d pt_target;
		std::tie(found, pt_target, std::ignore) = findBoxPoint(followed_model_ptr_);

		// generate path plan
		if ( found ) {
			if ( tryToApplyTarget(pt_target) ) {
				return (true);
			}
		}
		return (false);

	}

	// no action necessary at the moment
	return (true);

}

// ------------------------------------------------------------------- //

bool Target::stopFollowing() {

	// clear followed model pointer and `is_following` flag
	followed_model_ptr_ = nullptr;
	if ( is_following_ ) {

		// reset internal state
		is_following_ = false;
		has_target_ = false;
		has_global_plan_ = false;
		return (true);

	}
	return (false);

}

// ------------------------------------------------------------------- //

// `force_queue` false by default
bool Target::setNewTarget(const ignition::math::Vector3d &position, bool force_queue) {

	if ( force_queue ) {
		// add the target to the queue
		target_queue_.push(position);
	}

	// check validity of the position
	if ( std::isinf(position.X()) ||  std::isnan(position.X()) ||
		 std::isinf(position.Y()) ||  std::isnan(position.Y()) ) {
		return (false);
	}

	// check reachability - threshold selected experimentally
	// based on section 6. at `http://wiki.ros.org/costmap_2d`;
	// NOTE: costmap must be accessible to check whether
	// a certain cell is free (i.e. ROS must be running)
	if ( global_planner_.getCost(position.X(), position.Y()) > 100 ) {
		return (false);
	}

	// Check if `has_target_` flag is set. If no target is selected
	// then try to generate a path plan for a given position (`position` input).
	// If a valid plan could be generated, then save the point coordinates
	// as a target and update class' internal state.

	if ( !has_target_ ) {

		return (tryToApplyTarget(position));

	} else {

		// NOTE: the target is added to the queue without asking
		// the global planner whether a valid plan could be generated;
		// this action will be performed after the certain target
		// will be the first in the queue

		// add the target to the queue
		target_queue_.push(position);

	}

	return (true);

}

// ------------------------------------------------------------------- //

bool Target::setNewTarget(const ignition::math::Pose3d &pose) {

	// let the `setNewTarget(Vector3d)` take care of everything
	return (setNewTarget(pose.Pos()));

}

// ------------------------------------------------------------------- //

bool Target::setNewTarget(const std::string &object_name) {

	/* Find the closest point that belongs to space taken by object
	 * and try to artificially move that point to a closest free space
	 * location */
	bool is_valid = false;
	gazebo::physics::ModelPtr model;
	std::tie(is_valid, model) = isModelValid(object_name);

	if ( !is_valid ) {
		return (false);
	}

	// bounding box problems (actor only)
	if ( model->GetType() == actor::ACTOR_MODEL_TYPE_ID ) {
		/* not allowable, actor is a dynamic model;
		 * use `follow object` command instead */
		return (false);
	}

	// temp
	ignition::math::Vector3d pt_intersection;

	// return status according to `setNewTarget` operation success/failure
	return (setNewTarget(ignition::math::Pose3d(pt_intersection, model->WorldPose().Rot())));

}

// ------------------------------------------------------------------- //

bool Target::setNewTarget() {

	if ( target_queue_.empty() ) {
		return (false);
	}
	setNewTarget(target_queue_.front());
	target_queue_.pop();

	return (true);

}

// ------------------------------------------------------------------- //

bool Target::chooseNewTarget() {

	// FIXME: watch out for a situation in which actor's position is not in map bounds!
	// THIS IN FACT SHOULD NOT EVEN HAPPEN after some errors will be eliminated

	// FIXME: dynamic reconfiguration - any change hangs `global_planner_.makePlan` see `generatePathPlan`
	// check if costmap ready or sth? or check status of the error message

	// check whether global costmap has already initialized so global plan can be generated
	if ( !global_planner_.isCostmapInitialized() ) {
		// indicate that target can not be found now
		return (false);
	}

	ignition::math::Vector3d new_target(target_);
	bool reachable_gp = false; // whether global planner found a valid plan

	// 1) look for target that is located at least 2 meters from the current one
	// 2) look for target that is reachable (global plan can be found)
//	while ( ((new_target - target_).Length() < 2.0) || !reachable_gp ) {

	while ( ((new_target - target_).Length() < (2.0 * params_ptr_->getActorParams().target_tolerance))
			|| !reachable_gp )
	{

		// get random coordinates based on world limits
		new_target.X(ignition::math::Rand::DblUniform( params_ptr_->getActorParams().world_bound_x.at(0), params_ptr_->getActorParams().world_bound_x.at(1)) );
		new_target.Y(ignition::math::Rand::DblUniform( params_ptr_->getActorParams().world_bound_y.at(0), params_ptr_->getActorParams().world_bound_y.at(1)) );

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

			/* bounding-box-based target selection - safer for big obstacles,
			 * accounting some tolerance for a target accomplishment - an actor should
			 * not step into an object;
			 * also, check if current model is not listed as neglible */

			if ( isModelNegligible(world_ptr_->ModelByIndex(i)->GetName(), params_ptr_->getSfmDictionary().ignored_models_) ) {
//				std::cout << "MODEL NEGLIGIBLE: " << world_ptr_->ModelByIndex(i)->GetName() << "\tfor " << actor_ptr_->GetName() << std::endl;
				continue;
			}

			// check if model's bounding box contains target point
			if ( doesBoundingBoxContainPoint(world_ptr_->ModelByIndex(i)->BoundingBox(), new_target) ) {
				// TODO: make this an error log message
				std::cout << "chooseNewTarget() - selection failed -> model containing target's pos: " << world_ptr_->ModelByIndex(i)->GetName() << std::endl;
				std::cout << std::endl;
				new_target = target_;
				continue;
			}

			/* TODO: choose a target that is at least 1 meter from any obstacle ??? */

		} // for

		// seems that a proper target has been found, check whether it is reachable according to a global planner
		if ( generatePathPlan(new_target) ) {
			reachable_gp = true;
		} else {
			reachable_gp = false;
		}

	} // while

	// finally found a new target
	target_ = new_target;
	target_checkpoint_ = global_planner_.getWaypoint().Pos();

	// update state
	has_target_ = true;
	has_global_plan_ = true;

	// save event time
	time_last_target_selection_ = world_ptr_->SimTime();

	// Update checkpoint. After choosing a new target, the first checkpoint
	// is equal (nearly) to the current position. This may generate a problem
	// while actor is trying to align with target direction.
	// updateCheckpoint() will set temporary target to some further position
	// than the one actor is currently in.
	updateCheckpoint();

	// indicate success
	return (true);

}

// ------------------------------------------------------------------- //

// FIXME: name - update target?
bool Target::changeTarget() {

	if ( !isTargetQueueEmpty() ) {

		// pop a target from the queue
		if ( !setNewTarget() ) {
			return (false);
		}

		// after setting a new target, firstly let's rotate to its direction

	} else if ( chooseNewTarget() ) {

		// a completely new target has been chosen;
		// after setting new target, first let's rotate to its direction

	} else {

		// queue empty or target could not has been chosen
		return (false);

	}

	return (true);

}

// ------------------------------------------------------------------- //

bool Target::generatePathPlan(const ignition::math::Vector3d &target_to_be) {

	// Trying to find a global plan
	actor::ros_interface::GlobalPlan::MakePlanStatus status = actor::ros_interface::GlobalPlan::GLOBAL_PLANNER_UNKNOWN;

	size_t tries_num = 0;

	// repeat up to 10 times (more than 1 execution will be performed only when planner is busy)
	while ( tries_num++ <= 10 ) {

		std::cout << "\n\n\n\n\n[generatePathPlan] Starting iteration number " << tries_num << std::endl;

		// try to make a plan
		status = global_planner_.makePlan(pose_world_ptr_->Pos(), target_to_be);;

		// check action status
		switch (status) {

		case(actor::ros_interface::GlobalPlan::GLOBAL_PLANNER_SUCCESSFUL):
			std::cout << "\n\n\n[generatePathPlan] Global planning successfull\n\n\n" << std::endl;

			has_global_plan_ = true;
			return (true);
			break;

		case(actor::ros_interface::GlobalPlan::GLOBAL_PLANNER_FAILED):
			// TODO: check if within map bounds
			std::cout << "\n\n\n[generatePathPlan] Global planning failed\n\n\n" << std::endl;
			return (false);
			break;

		case(actor::ros_interface::GlobalPlan::GLOBAL_PLANNER_BUSY):
			// TODO: debug this
			std::cout << "\n\n\n[generatePathPlan] OOPS, need to wait for the global planner...\n\n\n" << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
			break;

		default:
			// unexpected behavior
			std::cout << "\n\n\n[generatePathPlan] UNEXPECTED BEHAVIOR\n\n\n" << std::endl;
			return (false);
			break;

		}

	}

	// if managed to get there then many tries were performed but planner is still busy
	std::cout << "\n\n\n[generatePathPlan] PLANNER UNABLE TO PROCESS THE REQUEST\n\n\n" << std::endl;
	return (false);

}

// ------------------------------------------------------------------- //
void Target::abandonTarget() {
	has_target_ = false;
	has_global_plan_ = false;
}

// ------------------------------------------------------------------- //
void Target::updateCheckpoint() {
	// take next checkpoint from vector (path)
	target_checkpoint_ = global_planner_.getWaypoint().Pos();
}

// ------------------------------------------------------------------- //
bool Target::isCostmapInitialized() {
	return (global_planner_.isCostmapInitialized());
}
// ------------------------------------------------------------------- //
bool Target::isPlanGenerated() const {
	return (has_global_plan_);
}
// ------------------------------------------------------------------- //
bool Target::isTargetChosen() const {
	return (has_target_);
}
// ------------------------------------------------------------------- //

bool Target::isTargetStillReachable() {

	// firstly check whether some target is set
	if ( !has_target_ ) {
		return (true);
	}

	// TODO: generate global plan?
	// check periodically, no need to do this in each iteration
	if ( (world_ptr_->SimTime() - time_last_reachability_).Double() > params_ptr_->getActorParams().target_reachable_check_period ) {

		// save event time
		time_last_reachability_ = world_ptr_->SimTime();

		if ( !is_following_ ) {

			// non-following handler (simple target reaching operation)
			//
			// iterate over all models
			for (unsigned int i = 0; i < world_ptr_->ModelCount(); ++i) {

				// FIXME: cafe is a specific model that represents whole world
				if ( world_ptr_->ModelByIndex(i)->GetName() == "cafe" ) {
					continue;
				}

				// check if model's bounding box contains target point
				if ( doesBoundingBoxContainPoint(world_ptr_->ModelByIndex(i)->BoundingBox(), target_) ) {

					std::cout << "isTargetStillReachable()" << std::endl;
//					std::cout << "\t" << actor_ptr_->GetName() << "\tDETECTED TARGET UNREACHABLE!" << std::endl;
					std::cout << "\ttarget: " << target_ << "\tmodel containing: " << world_ptr_->ModelByIndex(i)->GetName() << std::endl;
					std::cout << std::endl;
					std::cout << std::endl;
					std::cout << std::endl;
					return (false);

				} /* doesContain */

			} /* for */

		} else {

			// object tracking handler
			//
			if ( followed_model_ptr_ == nullptr ) {
				return (false);
			}

			// iterate over all models, try to find the same model in the database
			for (unsigned int i = 0; i < world_ptr_->ModelCount(); ++i) {

				if ( world_ptr_->ModelByIndex(i) == followed_model_ptr_ ) {
					return (true);
					break;
				} /* if (pointer matches) */

			} /* for */

			// model of given name was not found (loop was not `broken`)
			return (false);

		}

	}

	return (true);

}

// ------------------------------------------------------------------- //

bool Target::isTargetNotReachedForTooLong() const {

	// firstly check whether some target is set
	if ( !has_target_ ) {
		return (false);
	}

	// object tracking does not have a time limit
	if ( is_following_ ) {
		return (false);
	}

	if ( (world_ptr_->SimTime() - time_last_target_selection_).Double() > params_ptr_->getActorParams().target_reach_max_time ) {

		std::cout << "isTargetNotReachedForTooLong()" << std::endl;
//		std::cout << "\t" << actor_ptr_->GetName() << "\tDETECTED TARGET UNREACHABLE IN FINITE TIME!" << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
		return (true);

	}
	return (false);

}

// ------------------------------------------------------------------- //

bool Target::isTargetReached() {

	// firstly check whether some target is set
	if ( !has_target_ ) {
		return (true);
	}

	// calculate a distance to a target
	double distance_to_target = (target_ - pose_world_ptr_->Pos()).Length();

	// choose a new target position if the actor has reached its current target
	/* the smaller tolerance the bigger probability that actor will
	 * step into some obstacle */
	if ( distance_to_target > params_ptr_->getActorParams().target_tolerance ) {
		return (false);
	}

	// Also, check whether whole path has been traveled (sometimes it may happen
	// that path goes near the wall which has to be skipped over to reach
	// the actual goal)
	if ( !global_planner_.isTargetReached() ) {
		return (false);
	}

	// otherwise reset `has_target_` and return true
	has_target_ = false;

	return (true);

}

// ------------------------------------------------------------------- //

bool Target::isTargetQueueEmpty() const {
	return (target_queue_.empty());
}

// ------------------------------------------------------------------- //

bool Target::isCheckpointReached() const {

	// firstly check whether some target is set
	if ( !has_target_ ) {
		return (true);
	}

	// as a threshold value of length choose half of the `target_tolerance`
	double dist_to_checkpoint = (pose_world_ptr_->Pos() - target_checkpoint_).Length();
	if ( dist_to_checkpoint < (params_ptr_->getActorParams().target_tolerance) ) { // was multiplied by 0.5 but this forces a little too high accuracy to be met with SFM
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

// FIXME: this does not need to be called in each iteration (twice per second is enough)
bool Target::isCheckpointAbandonable() const {

	// firstly check whether some target is set
	if ( !has_target_ ) {
		return (false);
	}

	// non-const method if uncommented
//	// make some time gap between evaluations
//	if ( (world_ptr_->SimTime() - time_last_abandonability_).Double() >= 1.0 ) {
//		// force the current checkpoint to stay active
//		return (false);
//	}
//
//	// update time stamp
//	time_last_abandonability_ = world_ptr_->SimTime();

	// NOTE: conversion to global coordinate system
	ignition::math::Angle actor_glob_orient(pose_world_ptr_->Rot().Yaw() - IGN_PI_2);
	actor_glob_orient.Normalize();

	ignition::math::Vector3d actor_checkpoint_v = pose_world_ptr_->Pos() - target_checkpoint_;
	ignition::math::Angle actor_checkpoint_ang(std::atan2(actor_checkpoint_v.Y(), actor_checkpoint_v.X()));

	ignition::math::Angle angle_diff = actor_glob_orient - actor_checkpoint_ang;
	angle_diff.Normalize();

	// lower threshold may have a great impact on actor behaviour while in crowd:
	// sometimes faster abandonment of the target can produce smoother transitions
	// when 2 actors going in the opposite directions;
	// V1: lower threshold = 70 degrees
	// V2: lower threshold = 45 degrees (usually abandoned all checkpoints in a certain position)
	if ( std::fabs(angle_diff.Radian()) > IGN_DTOR(60) && std::fabs(angle_diff.Radian()) <= IGN_DTOR(90) ) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //
ignition::math::Vector3d Target::getTarget() const {
	return (target_);
}
// ------------------------------------------------------------------- //
ignition::math::Vector3d Target::getCheckpoint() const {
	return (target_checkpoint_);
}
// ------------------------------------------------------------------- //
nav_msgs::Path Target::getPath() const {
	return (global_planner_.getPath());
}
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //
// --- static, public methods section -------------------------------- //
// ------------------------------------------------------------------- //

/*
bool Target::isModelNegligible(const std::string &object_name) {

	// name has to be exactly given in .YAML - NOT HANDY for multiple objects of the same type
	/*
	std::vector<std::string>::iterator it;
	it = std::find ( params_ptr_->getSfmDictionary().ignored_models_.begin(), params_ptr_->getSfmDictionary().ignored_models_.end(), object_name);
	if ( it != params_ptr_->getSfmDictionary().ignored_models_.end() ) {
		// element found in `ignored models` vector
		return (true);
	}
	return (false);

}
*/

// ------------------------------------------------------------------- //

bool Target::isModelNegligible(const std::string &object_name, const std::vector<std::string> &dictionary) {

	// copy constructor
	std::string object_name_trim(object_name);

	// trim last character is it's number (model of the same type numbering)
	while ( std::isdigit(object_name_trim.back())  ) {
		object_name_trim.pop_back();
	}

//	std::cout << "isModelNegligible()  -  name: " << object_name << "\tPATTERN TO FIND: " << object_name_trim << "\t";

	// name consisted from numbers only?
	if ( object_name_trim.length() == 0 ) {
		return (false);
	}

	// iterate through whole dictionary to find something that matches
	std::size_t found;
	for ( size_t i = 0; i < dictionary.size(); i++ ) {

		found = dictionary.at(i).find(object_name_trim);
		if ( found != std::string::npos ) {
			// found something similar
//			std::cout << "FOUND" << std::endl;
			return (true);
		}

	}

//	std::cout << "NOT FOUND" << std::endl;
	// iterated through whole dictionary and did not found a given pattern
	return (false);

}

// ------------------------------------------------------------------- //

bool Target::doesBoundingBoxContainPoint(const ignition::math::Box &bb, const ignition::math::Vector3d &pt) {

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

Target::~Target() {
	// TODO Auto-generated destructor stub
}
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //
// private
std::tuple<bool, gazebo::physics::ModelPtr> Target::isModelValid(const std::string &object_name) {

	// Gazebo::Physics::World - ModelByName() says:
	/// `\return A pointer to the Model, or NULL if no model was found.`
	gazebo::physics::ModelPtr model_p = world_ptr_->ModelByName(object_name);

	if ( model_p == NULL || model_p == nullptr ) {
		return ( std::make_tuple(false, nullptr) );
	}
	return ( std::make_tuple(true, model_p) );

}

// ------------------------------------------------------------------- //

/// @note Must not be const because of call to GlobalPlan::getCost
ignition::math::Vector3d Target::findReachableDirectionPoint(const ignition::math::Vector3d &pt_intersection,
		const unsigned int &quarter) {

	/*
//	ignition::math::Vector3d v(nan, nan, nan);
	ignition::math::Vector3d v(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	ignition::math::Vector3d v_temp = pt_intersection;
	const double DELTA = 0.05;
	double x_delta = 0.0;
	double y_delta = 0.0;

//	// I quarter
//	pt_intersection.X( pt_intersection.X() - 0.05 );
//	pt_intersection.Y( pt_intersection.Y() - 0.05 );
//
//	// II quarter
//	pt_intersection.X( pt_intersection.X() + 0.05 );
//	pt_intersection.Y( pt_intersection.Y() - 0.05 );
//
//	// III quarter
//	pt_intersection.X( pt_intersection.X() + 0.05 );
//	pt_intersection.Y( pt_intersection.Y() + 0.05 );
//
//	// IV quarter
//	pt_intersection.X( pt_intersection.X() - 0.05 );
//	pt_intersection.Y( pt_intersection.Y() + 0.05 );

	// determine deltas based on direction of the line (see SetNewTarget method)
	switch (quarter) {

	case(1):
		x_delta = -DELTA;
		y_delta = -DELTA;
		break;
	case(2):
		x_delta = +DELTA;
		y_delta = -DELTA;
		break;
	case(3):
		x_delta = +DELTA;
		y_delta = +DELTA;
		break;
	case(4):
		x_delta = -DELTA;
		y_delta = +DELTA;
		break;

	}

	// perform `num_tries` checks moving away from the intersection point
	unsigned int num_tries = 100;
	while (num_tries--) {

		// check points located further and further away
		v_temp.X(v_temp.X() + x_delta);
		v_temp.Y(v_temp.Y() + y_delta);

		// if cost is small enough - accept the point and break the loop
		if ( global_planner_.getCost(v_temp.X(), v_temp.Y()) < 100 ) {
			v = pt_intersection;
			break;
		}

	}

	return (v);
	*/

	return (ignition::math::Vector3d());

}

// ------------------------------------------------------------------- //

bool Target::tryToApplyTarget(const ignition::math::Vector3d &pt) {

	// try to generate global path plan
	if ( generatePathPlan(pt) ) {

		// plan has been generated
		target_ = pt;
		target_checkpoint_ = global_planner_.getWaypoint().Pos();

		// Update checkpoint. After choosing a new target, the first checkpoint
		// is equal (nearly) to the current position. This may generate a problem
		// while actor is trying to align with target direction.
		// updateCheckpoint() will set temporary target to some further position
		// than the one actor is currently in.
		updateCheckpoint();

		// save event time
		time_last_target_selection_ = world_ptr_->SimTime();

		// update internal state
		has_global_plan_ = true;
		has_target_ = true;
		return (true);

	}
	return (false);

}

// ------------------------------------------------------------------- //

// non-const due to `getCost` call
std::tuple<bool, ignition::math::Vector3d, ignition::math::Vector3d> Target::findBoxPoint(const gazebo::physics::ModelPtr model_ptr) {

	// evaluate if bounding box is valid
	if ( model_ptr->BoundingBox().Size().Length() == 0.0 ||
		 std::isinf(model_ptr->BoundingBox().Size().Length()) ||
		 std::isnan(model_ptr->BoundingBox().Size().Length()) )
	{
		return (std::make_tuple(false, ignition::math::Vector3d(), ignition::math::Vector3d()));
	}

	// ========================================================================
//	/* check the line's direction and based on the angle shift
//	 * the intersection point a little in proper direction */
//	double line_angle = std::atan2(line.Direction().Y(), line.Direction().X());

	// ========================================================================
//	// evaluate bigger increase of the coordinate
//	if ( line.Direction().X() >= line.Direction().Y() ) {
//		// line's direction value is bigger (or equal) along the X axis
//	} else {
//		// line's direction value is bigger (or equal) along the Y axis
//	}

	// ========================================================================
	// ========================================================================
	// ========================================================================

	/* let's find the line from the current actor's pose to the closest
	 * point of an object's bounding box; shift the point to the free
	 * space direction a little and set it as a new target;
	 * this way a rise of an `unreachable target` flag won't occur,
	 * because target is located in a free space) */
	ignition::math::Line3d line;

	// Bounding-Box'es Intersect method output store (tuple)
	ignition::math::Vector3d pt_intersection;
	bool does_intersect = false;

	// costmap's cell may be problematic (possible collision) - let's try to reach
	// the model from the opposite side (along the calculated line)
	ignition::math::Vector3d pt_helper;

	// perform computations 2 times
	for ( size_t i = 0; i < 2; i++ ) {

		if ( i == 0 ) {

			// line_angle expressed from the actor to an object
			line.Set( pose_world_ptr_->Pos().X(), pose_world_ptr_->Pos().Y(), 0.0,
					  model_ptr->WorldPose().Pos().X(), model_ptr->WorldPose().Pos().Y(), 0.0 );

		} else if ( i == 1 ) {

			// create a helper point which probably help finding
			// another point of intersection (reachable from the other side);
			//
			// NOTE: `pt_intersection` is now (i==1) equal to the edge point
			// of the bounding box that is closest to the actor's center
			// (or is nearly the closest, no optimization here)
			pt_helper.X(pt_intersection.X() + 30.0 * line.Direction().X());
			pt_helper.X(pt_intersection.Y() + 30.0 * line.Direction().Y());

			// update line points, maintain line direction!
			line.Set(model_ptr->WorldPose().Pos().X(), model_ptr->WorldPose().Pos().Y(), 0.0,
					 pt_helper.X(), pt_helper.Y(), 0.0);

		}

		/* NOTE:
		 * line.Set( model->WorldPose().Pos(), pose_world_.Pos() );
		 * with commented version line_angle is expressed from object to actor,
		 * but for some reason Box always return pt of intersection equal
		 * to BoundingBox'es center (also, all 0.05 signs have to be inverted
		 * in below `if` conditions) */

		// evaluate intersection point (line and bounding box)
		std::tie(does_intersect, std::ignore, pt_intersection) = model_ptr->BoundingBox().Intersect(line);

		if ( !does_intersect ) {
			// this should not happen, something went wrong
			return (std::make_tuple(false, ignition::math::Vector3d(), ignition::math::Vector3d()));
			break;
		}

		// verify whether the point is reachable in terms of costmap
		if ( global_planner_.getCost(pt_intersection.X(), pt_intersection.Y()) < 100 ) {
			return (std::make_tuple(true, pt_intersection, line.Direction()));
			break;
		}

	} /* for loop end */

	// both intersections were found but the cost is too big - return
	// point of the last intersection and direction of the line
	return (std::make_tuple(false, pt_intersection, line.Direction()));

}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
