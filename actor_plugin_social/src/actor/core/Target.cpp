/*
 * Target.cpp
 *
 *  Created on: Jul 4, 2019
 *      Author: rayvburn
 */

#include <actor/core/Enums.h> 	// actor model id
#include <actor/core/Target.h>
#include <ignition/math/Rand.hh> // DblUniform
#include <thread>			// thread: sleep for
#include <chrono>			// timer: sleep_for

namespace actor {
namespace core {

// ------------------------------------------------------------------- //
Target::Target(): has_target_(false), has_global_plan_(false), has_new_path_(true), is_following_(false),
		is_followed_object_reached_(false), followed_model_ptr_(nullptr) { }
// ------------------------------------------------------------------- //

Target::Target(gazebo::physics::WorldPtr world_ptr, std::shared_ptr<const ignition::math::Pose3d> pose_world_ptr,
			   std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr)
					: has_target_(false), has_global_plan_(false), has_new_path_(false),
					  is_following_(false), is_followed_object_reached_(false), followed_model_ptr_(nullptr) {

	world_ptr_ = world_ptr;
	pose_world_ptr_ = pose_world_ptr;
	params_ptr_ = params_ptr;

}

// ------------------------------------------------------------------- //

Target::Target(const Target &obj) {

	// copy ctor
	has_target_ = obj.has_target_;
	has_global_plan_ = obj.has_global_plan_;
	has_new_path_ = obj.has_new_path_;
	is_following_ = obj.is_following_;
	is_followed_object_reached_ = obj.is_followed_object_reached_;

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

	// will not allow following objects from the areas
	// with high cost (in terms of costmap) or just static
	// obstacles that are already marked on the actor costmap
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

bool Target::isFollowedObjectReached() const {
	if ( isFollowing() ) {
		return (is_followed_object_reached_);
	}
	return (false);
}

// ------------------------------------------------------------------- //

bool Target::updateFollowedTarget() {

	// check model's pointer validity (this is an operation which does not consume
	// much resources so can be performed in each iteration (opposite to a global path generation))
	if ( followed_model_ptr_ == nullptr ) {
		is_following_ = false;
		is_followed_object_reached_ = false;
		has_target_ = false;
		has_global_plan_ = false;
		has_new_path_ = false;
		return (false);
	}

	// V1: periodically update the tracked object's position
	//if ( (world_ptr_->SimTime() - time_last_follow_plan_).Double() >= 2.0 ) {

	// V2: periodically update the tracked object's position if it does move
	if ( (world_ptr_->SimTime() - time_last_follow_plan_).Double() >= 2.0 &&
		 (target_ - followed_model_ptr_->WorldPose().Pos()).Length() > (0.25 * params_ptr_->getActorParams().target_tolerance)) {

		// debugging model oscillations:
		// FIXME: about 0.22 m for a completely static object? is that normal? ODE?
		//std::cout << "oscillation range: " << (target_ - followed_model_ptr_->WorldPose().Pos()).Length() << std::endl;

		// update event time
		time_last_follow_plan_ = world_ptr_->SimTime();

		// do not generate a new global plan if object is not moving;
		// NOTE: for some reason static Turtlebot's velocity is:
		// [0.003461 0.000151 -0.00718]
		// FIXME: debugging with a static bot
//		if ( followed_model_ptr_->WorldLinearVel().Length() <= 0.01 ) {
//			return (true);
//		}

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
		is_followed_object_reached_ = false;
		has_target_ = false;
		has_global_plan_ = false;
		has_new_path_ = false;
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
		return (true);

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

	// try to find line-box intersection point
	ignition::math::Vector3d pt_target;
	ignition::math::Vector3d dir;
	std::tie(std::ignore, pt_target, dir) = findBoxPoint(model); // `found` will be false for sure - object's edge (cost > 250)

	// try to find a safe point near the target object whose
	// cost (according to the global planner is small enough)
	int tries_num = 10; // along with 0.5 factor below - maximum move-away from the intersection point is 5 m
	while (tries_num--) {

		if ( global_planner_.getCost(pt_target.X(), pt_target.Y()) > 100 ) {
			// move the point a little further according to line direction;
			// move in the opposite direction (towards actor going from
			// the obstacle)
			pt_target.X(pt_target.X() - 0.5 * dir.X());
			pt_target.Y(pt_target.Y() - 0.5 * dir.Y());
			continue;
		}

		return (setNewTarget(pt_target));

	}

	// a point with a low cost was not found
	return (false);

}

// ------------------------------------------------------------------- //

bool Target::setNewTarget() {

	if ( target_queue_.empty() ) {
		return (false);
	}
	setNewTarget(target_queue_.front());

	// NOTE: sometimes crash occur while popping without evaluation if empty.
	// Multithreading? TODO: mutex?
	if ( !target_queue_.empty() ) {
		target_queue_.pop();
	}
	return (true);

}

// ------------------------------------------------------------------- //

bool Target::chooseNewTarget() {

	// check whether global costmap has already initialized so global plan can be generated
	if ( !global_planner_.isCostmapInitialized() ) {
		// indicate that target can not be found now
		return (false);
	}

	// Watch out for a situation in which actor's position is not within map bounds.
	// A flag indicating that the actor is located in a place with a high
	// cost (like he is unable to start in terms of cost).
	// NOTE: Despite the fact that the costmap's `trinary_costmap` parameter
	// was set to `false` the costmap returns only these cost values:
	// 	o 	0 (free space),
	//  o	253 (unsafe space around the obstacle),
	// 	o 	254 (lethal obstacle).
	bool extend_safe = false;
	ignition::math::Vector3d start_safe = pose_world_ptr_->Pos();
	if ( global_planner_.getCost(pose_world_ptr_->Pos().X(), pose_world_ptr_->Pos().Y()) != 0 ) { // >0 or -1
		extend_safe = true;
	}

	ignition::math::Vector3d new_target(target_);
	bool reachable_gp = false; // whether global planner found a valid plan

	// Target selection conditions:
	// 1) look for target that is located at least 2 x TARGET_TOLERANCE [m] from the current one
	// 2) look for target that is reachable (global plan can be found)
	while ( !((new_target - target_).Length() >= (2.0 * params_ptr_->getActorParams().target_tolerance) && reachable_gp) ) {

		// reset flag
		reachable_gp = false;

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
			 * also, check if current model is not marked as negligible */
			if ( isModelNegligible(world_ptr_->ModelByIndex(i)->GetName(), params_ptr_->getSfmDictionary().ignored_models_) ) {
//				std::cout << "MODEL NEGLIGIBLE: " << world_ptr_->ModelByIndex(i)->GetName() << std::endl;
				continue;
			}

			// check if model's bounding box contains target point
			if ( doesBoundingBoxContainPoint(world_ptr_->ModelByIndex(i)->BoundingBox(), new_target) ) {
				std::cout << "chooseNewTarget() - selection failed -> model containing target's pos: " << world_ptr_->ModelByIndex(i)->GetName() << std::endl;
				std::cout << std::endl;
				new_target = target_; // find another
				continue;
			}

		} // for

		// point seems to be valid, but its cost must be evaluated if the actor's
		// current position is somewhere in the high-cost location
		if ( extend_safe ) {
			bool found_safe = false;
			std::tie(found_safe, start_safe) = findSafePositionAlongLine(pose_world_ptr_->Pos(), new_target);
			//std::cout << "found_safe: " << found_safe << "\tstart: " << pose_world_ptr_->Pos() << "\tstart_shift: " << start_safe;
			if ( !found_safe ) {
				start_safe = pose_world_ptr_->Pos(); // not found - bring back the world position - TROUBLE on the way!
				//std::cout << "\tNOT APPLIED" << std::endl;
				std::cout << "[ERROR] actor's global planner cannot find a valid path because the actor's position has a high cost according to the costmap. System is probably stuck now :)" << std::endl;
			} else {
				//std::cout << "\tAPPLIED!" << std::endl;
			}
		}

		// seems that a proper target has been found, check whether it is reachable according to a global planner
		if ( generatePathPlan(start_safe, new_target) ) {
			reachable_gp = true;
			break; // necessary to break the `while` loop - seems that a valid new target was found and a path has been generated successfully
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
	has_new_path_ = true;

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

bool Target::generatePathPlan(const ignition::math::Vector3d &start, const ignition::math::Vector3d &target_to_be) {

	// Trying to find a global plan
	actor::ros_interface::GlobalPlan::MakePlanStatus status = actor::ros_interface::GlobalPlan::GLOBAL_PLANNER_UNKNOWN;

	size_t tries_num = 0;

	// repeat up to 10 times (more than 1 execution will be performed only when planner is busy)
	while ( tries_num++ <= 10 ) {

		std::cout << "\n\n\n\n\n[generatePathPlan] Starting iteration number " << tries_num << std::endl;

		// try to make a plan
		status = global_planner_.makePlan(start, target_to_be);;

		// check action status
		switch (status) {

		case(actor::ros_interface::GlobalPlan::GLOBAL_PLANNER_SUCCESSFUL):
			std::cout << "\n\n\n[generatePathPlan] Global planning successfull\n\n\n" << std::endl;

			has_global_plan_ = true;
			has_new_path_ = true;
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
	has_new_path_ = false;
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

	// check periodically, no need to do this in each iteration
	if ( (world_ptr_->SimTime() - time_last_reachability_).Double() > params_ptr_->getActorParams().target_reachable_check_period ) {

		// save event time
		time_last_reachability_ = world_ptr_->SimTime();

		if ( !is_following_ ) {

			// non-following handler (simple target reaching operation)
			//
			// iterate over all models
			for (unsigned int i = 0; i < world_ptr_->ModelCount(); ++i) {

				// ignore specific models that cover whole world
				// or are listed as `negligible`
				if ( isModelNegligible( world_ptr_->ModelByIndex(i)->GetName(),
										params_ptr_->getSfmDictionary().ignored_models_) )
				{
					continue;
				}

				// check if model's bounding box contains target point
				if ( doesBoundingBoxContainPoint(world_ptr_->ModelByIndex(i)->BoundingBox(), target_) ) {

					std::cout << "isTargetStillReachable()" << std::endl;
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
		std::cout << "curr_time: " << world_ptr_->SimTime().Double() << "\tlast: " << time_last_target_selection_.Double() << "\tdelta_cur: " << (world_ptr_->SimTime() - time_last_target_selection_).Double() << "\tmax: " << params_ptr_->getActorParams().target_reach_max_time << std::endl;
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

	/* NOTE: there is a problem related to the oscillations of the animation position
	 * while an actor is standing and not moving. This issue is negligible
	 * when actor is operating in `target reachment` mode. But if `object tracking`
	 * is active an issue described below occurs.
	 * Once in `object tracking` mode, `distance_to_target` may be below the given
	 * threshold (object stopped or sth). The actor's animation immediately changes -
	 * from `walk` to `stand`.
	 * While `stand` is active, the position oscillates and may exceed
	 * the threshold distance (`target_tolerance`) which makes detection of
	 * renewed object's motion impossible (`is_followed_object_reached_` set
	 * to false despite of the fact that neither the actor nor the object
	 * moved explicitly.
	 * Let's artificially increase the target tolerance in object tracking mode.
	 */
	double track_factor = 1.0;
	if ( is_following_ && is_followed_object_reached_ ) { // although these 2 flags coexist, they are put together just in case
		// is_followed_object_reached_ is true so the target has been reached
		// and actor likely stands in front of it waiting until it moves somewhere
		track_factor = 1.05;
	}

	// choose a new target position if the actor has reached its current target
	/* the smaller tolerance the bigger probability that actor will
	 * step into some obstacle */
	if ( distance_to_target > (track_factor * params_ptr_->getActorParams().target_tolerance) ) {
		if ( is_following_ ) {
			is_followed_object_reached_ = false;
		}
		return (false);
	}

	// Also, check whether whole path has been traveled (sometimes it may happen
	// that path goes near the wall which has to be skipped over to reach
	// the actual goal)
	if ( !global_planner_.isTargetReached() ) {
		if ( is_following_ ) {
			is_followed_object_reached_ = false;
		}
		return (false);
	}

	// the target location is reached, check `is_following` flag to decide what to do next
	if ( is_following_ ) {
		// do not abandon the target if following some object
		is_followed_object_reached_ = true;
	} else {
		// otherwise reset `has_target_` and return true
		has_target_ = false;
	}

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

bool Target::isCheckpointAbandonable() const {

	// firstly check whether some target is set
	if ( !has_target_ ) {
		return (false);
	}

	// non-const method if uncommented
	// make some time gap between evaluations, 100 Hz refresh rate
//	if ( (world_ptr_->SimTime() - time_last_abandonability_).Double() >= 0.01 ) {
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

bool Target::isPathNew() {
	bool temp = has_new_path_;
	has_new_path_ = false;
	return (temp);
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

		// - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// V1
//		// check if model's bounding box contains target point
//		if ( bb.Contains(pt) ) {
//			return (true);
//		}
		// - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// V2
		// try to inflate the BB and then check if it does contain;
		// let the inflation size be a `target_tolerance` parameter
		// multiplied by 0.5 -> NOT exactly, static fcn so param cannot
		// be read explicitly, 0.5 * DEFAULT VALUE of the parameter
		ignition::math::Box bb_inf = bb;
		bb_inf.Max().X(bb_inf.Max().X() + 0.3125);
		bb_inf.Max().Y(bb_inf.Max().Y() + 0.3125);

		bb_inf.Min().X(bb_inf.Min().X() - 0.3125);
		bb_inf.Min().Y(bb_inf.Min().Y() - 0.3125);

		if ( bb_inf.Contains(pt) ) {
			return (true);
		}
		// - - - - - - - - - - - - - - - - - - - - - - - - - - -

	}
	return (false);

}

// ------------------------------------------------------------------- //

Target::~Target() {}

// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //
// public
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

std::tuple<bool, ignition::math::Vector3d> Target::findSafePositionAlongLine(const ignition::math::Vector3d &start,
		const ignition::math::Vector3d &end, const double &min_dist_to_end)
{

	ignition::math::Line3d line;
	line.Set(start.X(), start.Y(), 0.0, end.X(), end.Y(), 0.0);
	ignition::math::Vector3d line_dir = line.Direction();
	ignition::math::Vector3d start_shifted = start;

	// try to find a safe point near the target object whose
	// cost (according to the global planner is small enough)
	int tries_num = 10; // along with 0.5 factor below - maximum move-away from the intersection point is 5 m
	while (tries_num--) {

		// for some reason the getCost method returns unstable results (for the same point
		// it shows 253 in first check and 0 in the next (actually invalid))
		int16_t surrounding[4];
		surrounding[0] = global_planner_.getCost(start_shifted.X()-0.01, start_shifted.Y()-0.01);
		surrounding[1] = global_planner_.getCost(start_shifted.X()-0.01, start_shifted.Y()+0.01);
		surrounding[2] = global_planner_.getCost(start_shifted.X()+0.01, start_shifted.Y()-0.01);
		surrounding[3] = global_planner_.getCost(start_shifted.X()+0.01, start_shifted.Y()+0.01);
		int16_t cost_superpose = *std::max_element(surrounding, surrounding+4);

		if ( cost_superpose > 0 ) {
			// move the point a little further according to line direction;
			// move in the opposite direction (towards obstacle starting from
			// the actor)
			start_shifted.X(start_shifted.X() + 0.5 * line_dir.X());
			start_shifted.Y(start_shifted.Y() + 0.5 * line_dir.Y());
			continue;
		}

		// check how far from the end point the shifted `start` point is located
		if ( (end - start_shifted).Length() >= min_dist_to_end ) {
			return (std::make_tuple(true, start_shifted));
		} else {
			return (std::make_tuple(false, start_shifted));
		}

	}
	return (std::make_tuple(false, ignition::math::Vector3d()));

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
	if ( generatePathPlan(pose_world_ptr_->Pos(), pt) ) {

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
		resetTimestamps(); // does -> | time_last_target_selection_ = world_ptr_->SimTime(); | and even more

		// update internal state
		has_global_plan_ = true;
		has_new_path_ = true;
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
			line.Set(pose_world_ptr_->Pos().X(), pose_world_ptr_->Pos().Y(), model_ptr->BoundingBox().Center().Z(),
					 model_ptr->BoundingBox().Center().X(), model_ptr->BoundingBox().Center().Y(), model_ptr->BoundingBox().Center().Z());

		} else if ( i == 1 ) {

			// create a helper point which probably help finding
			// another point of intersection (reachable from the other side);
			//
			// NOTE: `pt_intersection` is now (i==1) equal to the edge point
			// of the bounding box that is closest to the actor's center
			// (or is nearly the closest, no optimization here)
			pt_helper.X(pt_intersection.X() + 30.0 * line.Direction().X());
			pt_helper.Y(pt_intersection.Y() + 30.0 * line.Direction().Y());

			// update line points, maintain line direction!
			line.Set(model_ptr->BoundingBox().Center().X(), model_ptr->BoundingBox().Center().Y(), model_ptr->BoundingBox().Center().Z(),
					 pt_helper.X(), pt_helper.Y(), model_ptr->BoundingBox().Center().Z());

		}

		/* NOTE:
		 * line.Set( model->WorldPose().Pos(), pose_world_.Pos() );
		 * with commented version line_angle is expressed from object to actor,
		 * but for some reason Box always return pt of intersection equal
		 * to BoundingBox'es center (also, all 0.05 signs have to be inverted
		 * in below `if` conditions) */

		// evaluate intersection point (line and bounding box)
		std::tie(does_intersect, std::ignore, pt_intersection) = model_ptr->BoundingBox().Intersect(line);
		pt_intersection.Z(0.0);

		if ( !does_intersect ) {
			// this should not happen, something went wrong;
			// it is noticeable that sometimes even though a model
			// is valid and line's direction seems to be ok,
			// `does_intersect` stays FALSE and pt_intersection is (0,0,0)
			return (std::make_tuple(false, ignition::math::Vector3d(), ignition::math::Vector3d()));
			break;
		}

		// verify whether the point is reachable in terms of costmap
		int16_t cost = global_planner_.getCost(pt_intersection.X(), pt_intersection.Y());
		if ( cost < 100 && cost >= 0 ) {
			return (std::make_tuple(true, pt_intersection, line.Direction()));
			break;
		} else {
			// intersection point has been found and even though cost is big,
			// let's send intersection point and the line's direction to the caller
			return (std::make_tuple(false, pt_intersection, line.Direction()));
			break;
		}

	} /* for loop end */

	// both intersections were found but the cost is too big - return
	// point of the last intersection and direction of the line
	return (std::make_tuple(false, pt_intersection, line.Direction()));

}

// ------------------------------------------------------------------- //

void Target::resetTimestamps() {

	time_last_target_selection_ = world_ptr_->SimTime();
	time_last_reachability_ = world_ptr_->SimTime();
//	time_last_abandonability_ = world_ptr_->SimTime();
	time_last_follow_plan_ = world_ptr_->SimTime();

}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
