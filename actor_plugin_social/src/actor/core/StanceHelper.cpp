/*
 * StanceHelper.cpp
 *
 *  Created on: Mar 5, 2020
 *      Author: rayvburn
 */

#include <actor/core/StanceHelper.h>

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

StanceHelper::StanceHelper()
	: stance_status_(STANCE_UNKNOWN),
	  stance_(ACTOR_STANCE_UNKNOWN)
{}

// ------------------------------------------------------------------- //

void StanceHelper::init(const gazebo::physics::Actor::SkeletonAnimation_M &anims, const actor::ActorStance &stance_init) {

	skeleton_anims_ = anims;
	stance_ = stance_init;
	stance_status_ = STANCE_INITED;

	// create a temporary trajectory to prevent errors
	trajectory_info_ptr_.reset(new gazebo::physics::TrajectoryInfo());
	trajectory_info_ptr_->type = convertStanceToAnimationName(stance_init);
	trajectory_info_ptr_->duration = 1.0;

}

// ------------------------------------------------------------------- //

bool StanceHelper::configure(const std::queue<actor::ActorStance> &stance_type_v) {

	// treat as error
	if ( stance_type_v.size() == 0 ) {
		return (false);
	}

	// backup for later use in case of configuration procedure failure
	uint8_t stance_status_backup = stance_status_;
	actor::ActorStance stance_backup = stance_;
	std::queue<actor::ActorStance> stance_sequence_backup = stance_sequence_;

	// remove any previous sequence
	clearQueue();
	stance_status_ = STANCE_UNKNOWN;

	// iterate over vector elements
	for ( size_t i = 0; i < stance_type_v.size(); i++ ) {
		// check status
		if ( !configure(stance_type_v.front()) ) {
			// pop back newly added items - full sequence in or ignored totally
			// NOTE:
			// the first stance will not be copied into sequence
			if ( i > 0 ) {
				for ( size_t j = (i - 1); j >= 0; j-- ) {
					stance_sequence_.pop();
				}
			}
			stance_ = stance_backup;
			stance_sequence_ = stance_sequence_backup;
			stance_status_ = stance_status_backup;
			return (false);
			break; // for clarity
		}
	}
	return (true);

}

// ------------------------------------------------------------------- //

bool StanceHelper::configure(const actor::ActorStance &stance_type) {

	// NOTE: at the moment 2 consequent stances of the sequence can be the same!

	std::string animation = convertStanceToAnimationName(stance_type);

	/* To print available animations:
	for (auto& x: skeleton_anims_) { std::cout << "Skel. animation: " << x.first << std::endl; }
	*/

	if ( skeleton_anims_.find(animation) != skeleton_anims_.end() ) {

		// animation found, evaluate `stance_status_` now
		if ( stance_status_ == STANCE_EXECUTION || stance_status_ == STANCE_UNKNOWN ) {
			// seems that a new stance request is received

			// Create custom trajectory
			trajectory_info_ptr_.reset(new gazebo::physics::TrajectoryInfo());
			trajectory_info_ptr_->type = animation;
			if ( isDisposableAnimation(stance_type) ) {
				trajectory_info_ptr_->duration = 1.0;
			} else {
				trajectory_info_ptr_->duration = 10000.0; // std::numeric_limits<double>::infinity();
			}

			stance_ = stance_type;
			stance_status_ = STANCE_SELECTED;
			return (true);

		} else if ( stance_status_ == STANCE_SELECTED ) {

			// a new stance has just been selected but was not executed yet,
			// let's queue the newly received stance
			stance_sequence_.push(stance_type);
			return (true);

		}


	} else {

		// print warning
		std::cout << "[StanceHelper::configure] Skeleton animation ''" << animation << "'' not found.\n";

	}
	return (false);

}

// ------------------------------------------------------------------- //

bool StanceHelper::update(const gazebo::common::Time& time) {

	// evaluate the current stance
	if ( stance_status_ == STANCE_INITED ) {

		std::cout << "[StanceHelper::update] inited" << std::endl;

		// need to configure at first
		if ( !configure(stance_) ) {
			stance_status_ = STANCE_UNKNOWN;
			return (false);
		}
		return (true);
		// do not return TRUE?

	} else if ( stance_status_ == STANCE_SELECTED ) {

		std::cout << "[StanceHelper::update] selected" << std::endl;

		// new request has just been received

		// NOTE: already configured
//		trajectory_end_time_.Set(time.Double() + trajectory_info_ptr_->duration);
		// update status
		stance_status_ = STANCE_EXECUTION;
		return (true);

	} else if ( stance_status_ == STANCE_EXECUTION ) {

//		std::cout << "[StanceHelper::update] execution" << std::endl;

		// evaluate whether the animation execution should be terminated
		if ( (trajectory_end_time_.Double() != std::numeric_limits<double>::infinity()) //&&
//			 (time.Double() >= trajectory_end_time_.Double()) ) {
		){
			// the next sequence element should be chosen (if exists)
			if ( stance_sequence_.size() > 0 ) {

				actor::ActorStance stance = stance_sequence_.front();
				stance_sequence_.pop();
				if ( configure(stance) ) {
					return (true);
				}

			}

		}
		return (false); // for clarity

	}
	return (false);

}

// ------------------------------------------------------------------- //

gazebo::physics::TrajectoryInfoPtr& StanceHelper::getTrajectoryInfoPtr() {
	return (trajectory_info_ptr_);
}

// ------------------------------------------------------------------- //

actor::ActorStance StanceHelper::getStance() const {
	return (stance_);
}

// ------------------------------------------------------------------- //

void StanceHelper::printDebugInfo(const std::string &owner_name) {

	// debug info
	std::cout << "[STANCE] " << owner_name << "'s\tnew stance:\t";
	if ( stance_ != ACTOR_STANCE_LIE ) {
		std::cout << trajectory_info_ptr_->type << std::endl;
	} else {
		// `lie` must be handled differently as the `lie` is equal to `stand`
		// but in a different plane
		std::cout << "lie" << std::endl;
	}

}

// ------------------------------------------------------------------- //

std::string StanceHelper::convertStanceToAnimationName(const actor::ActorStance &stance_type) {

	std::string anim_name;

	switch( stance_type ) {

	case(ACTOR_STANCE_WALK):
			anim_name = "walk";
			break;
	case(ACTOR_STANCE_STAND):
			anim_name = "stand";
			break;
	case(ACTOR_STANCE_LIE):
			anim_name = "stand"; // lie is stand but in different plane
			break;
	case(ACTOR_STANCE_SIT_DOWN):
			anim_name = "sit_down";
			break;
	case(ACTOR_STANCE_SITTING):
			anim_name = "sitting";
			break;
	case(ACTOR_STANCE_STAND_UP):
			anim_name = "stand_up";
			break;
	case(ACTOR_STANCE_RUN):
			anim_name = "run";
			break;
	case(ACTOR_STANCE_TALK_A):
			anim_name = "talk_a";
			break;
	case(ACTOR_STANCE_TALK_B):
			anim_name = "talk_b";
			break;
	default:
			break;

	}
	return (anim_name);

}

// ------------------------------------------------------------------- //

bool StanceHelper::isDisposableAnimation(const actor::ActorStance &stance_type) {

	switch( stance_type ) {

		case(ACTOR_STANCE_SIT_DOWN):
		case(ACTOR_STANCE_STAND_UP):
			return (true);
		default:
			// e.g. `running` is of course a dynamic movement but can be played multiple
			// times and still look natural (unless the actor is static then)
			return (false);
			break;

	}

}

// ------------------------------------------------------------------- //

void StanceHelper::clearQueue() {

	size_t len = stance_sequence_.size();
	for ( size_t i = 0; i < len; i++ ) {
		stance_sequence_.pop();
	}

}

// ------------------------------------------------------------------- //

StanceHelper::~StanceHelper() {}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
