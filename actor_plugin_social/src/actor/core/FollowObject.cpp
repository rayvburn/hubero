/*
 * FollowObject.cpp
 *
 *  Created on: Jan 2, 2020
 *      Author: rayvburn
 */

#include <actor/core/FollowObject.h>
#include <thread>
#include <chrono>

namespace actor {
namespace core {

FollowObject::FollowObject(): path_calc_tries_num_(10) {}

void FollowObject::start() {
	PTF::start();
	path_calc_tries_num_ = 10;
}

void FollowObject::execute(const gazebo::common::Time &curr_time) {

	// check if call is executed for a proper mode
	if ( !this->target_manager_ptr_->isFollowing() ) {
		this->status_int_ = NOT_FOLLOWING;
		this->terminal_flag_ = true;
		return;
	}

	// helper flag
	bool stop_tracking = false;

	// flags helping in evaluation whether the tracked object started moving away again
	bool target_dir_alignment = false;	// whether to change the state to `ALIGN_WITH_TARGET_DIR`
	bool object_prev_reached = false; 	// by default the tracked object moves away
	if ( this->target_manager_ptr_->isFollowedObjectReached() ) {
		// let's save the previous status before performing any further
		// evaluations (for example `isTargetReached()`)
		object_prev_reached = true; // tracked object has been stopped so far
	}

	// try to update a global path leading the actor towards the dynamic object;
	// this also checks whether an object to follow is selected
	if ( !this->target_manager_ptr_->updateFollowedTarget() ) {
		// recently followed object is not reachable at the moment (object deleted
		// from the simulation or global plan cannot be found)

		// evaluate whether to wait for a more friendly configuration (object
		// location on the costmap) or immediately terminate the follow object state
		if ( doWait(curr_time) ) {

			this->status_int_ = WAIT_FOR_MOVEMENT;
			this->text_ = "tracked object is not reachable now, waiting...";
			return;

		} else {

			stop_tracking = true;
			this->status_int_ = UNABLE_TO_FIND_PLAN;
			this->text_ = "tracked object is not reachable anymore (has been deleted from the world or a global plan cannot be generated)";
			this->terminal_flag_ = true;
			return;

		}

	} else {

		this->status_int_ = TRACKING;
		this->text_ = "tracking";

	}

	// check closeness to the target/checkpoint
	if ( this->target_manager_ptr_->isTargetReached() ) {
		// actor is close enough to the tracked object (it may not be moving for some time);
		// do not call stopFollowing etc and do not change the state,
		// just do not try to go further at the moment
		this->status_int_ = TARGET_REACHED;
	} else if ( object_prev_reached ) {
		// detects change of the `is_followed_object_reached_`
		// flag (i.e. tracked object started moving again)
		target_dir_alignment = true;
//		action_info_.setStatus(actor::core::Action::PREPARING, "tracked object was static but started moving again, actor will rotate towards the object");
	} else if ( this->target_manager_ptr_->isCheckpointReached() ) {
		// take next checkpoint from vector (path)
		this->target_manager_ptr_->updateCheckpoint();
		this->status_int_ = TRACKING;
	}


	// check whether a current checkpoint is abandonable (angle-related)
	if ( this->target_manager_ptr_->isCheckpointAbandonable() ) {
		this->target_manager_ptr_->updateCheckpoint();
		this->status_int_ = TRACKING;
	}

	// check if the tracked object still exists in the world since last target selection
	if ( !this->target_manager_ptr_->isTargetStillReachable() ) {
		stop_tracking = true;
		this->status_int_ = NOT_REACHABLE;
//		action_info_.setStatus(actor::core::Action::OBJECT_NON_REACHABLE, "tracked object became unreachable");
	}

	// change FSM state if needed
	if ( stop_tracking ) {
		this->target_manager_ptr_->abandonTarget();
		this->target_manager_ptr_->stopFollowing();
//		ignored_models_.pop_back(); // FIXME: it won't work if ignored_models stores other elements than followed object's name
//		setState(ActorState::ACTOR_STATE_STOP_AND_STARE);
//		sfm_.reset(); // clear SFM markers
		this->terminal_flag_ = true;
		return;
	}

	// dynamic target has been reached, do not change the state, just change stance
	if ( this->target_manager_ptr_->isFollowedObjectReached() ) {
		// make actor stop;
		// when tracked object will start moving again, then stance will be changed
		this->status_int_ = WAIT_FOR_MOVEMENT;
//		setStance(ActorStance::ACTOR_STANCE_STAND);
//		// update the pose (stance only) because the update event will be broken (stopped)
//		updateStanceOrientation(*pose_world_ptr_);
//		applyUpdate(0.001);
//		action_info_.setStatus(actor::core::Action::OBJECT_REACHED, "tracked object is within 'reachment' tolerance range");
//		this->terminal_flag_ = true; // PREVIOUSLY WHY ???????????????
	}

	// target previously was reached but started moving again - let's align
	// with direction to its center
	if ( target_dir_alignment ) {
		this->status_int_ = ROTATE_TOWARDS_OBJECT;
//		setStance(ActorStance::ACTOR_STANCE_WALK);
//		setState(ActorState::ACTOR_STATE_ALIGN_TARGET);
		this->terminal_flag_ = true;
	}

//	this->terminal_flag_ = false;

}

FollowObject::~FollowObject() {}

bool FollowObject::doWait(const gazebo::common::Time &curr_time) {

	if ( (curr_time - time_last_path_calculation_).Double() < 2.0 ) {
		return (true);
	}

	// time update
	time_last_path_calculation_ = curr_time;

	if ( path_calc_tries_num_-- ) {
		return (true);
	} else {
		// enough...
		return (false);
	}

}

} /* namespace core */
} /* namespace actor */
