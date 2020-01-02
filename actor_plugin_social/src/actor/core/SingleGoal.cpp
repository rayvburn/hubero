/*
 * SingleGoal.cpp
 *
 *  Created on: Jan 2, 2020
 *      Author: rayvburn
 */

#include <actor/core/SingleGoal.h>

namespace actor {
namespace core {

SingleGoal::SingleGoal() {
	// TODO Auto-generated constructor stub

}

void SingleGoal::execute() {

	// check if call is executed for a proper mode
	if ( this->target_manager_ptr_->isFollowing() ) {
		return (false);
	}

	// helper flag
	bool abandon = false;

	// check whether a target exists
	if ( !this->target_manager_ptr_->isTargetChosen() ) {
		//return (false);
		abandon = true;
	}

	// check whether a target has plan generated
	if ( !this->target_manager_ptr_->isPlanGenerated() ) {
		if ( !this->target_manager_ptr_->generatePathPlan(this->target_manager_ptr_->getPose().Pos(), this->target_manager_ptr_->getTarget()) ) {
			abandon = true;
		}
	}

	// check whether a current checkpoint is abandonable
	if ( this->target_manager_ptr_->isCheckpointAbandonable() ) {
		this->target_manager_ptr_->updateCheckpoint();
	}

	// check closeness to the target/checkpoint
	if ( this->target_manager_ptr_->isTargetReached() ) {
		abandon = true;
	} else if ( this->target_manager_ptr_->isCheckpointReached() ) {
		// take next checkpoint from vector (path)
		this->target_manager_ptr_->updateCheckpoint();
	}

	// reachability test 1:
	// check if there has been some obstacle put into world since last target selection
	if ( !this->target_manager_ptr_->isTargetStillReachable() ) {
		abandon = true;
	}

	// reachability test 2:
	// check if actor is stuck
	if ( this->target_manager_ptr_->isTargetNotReachedForTooLong() ) {
		abandon = true;
	}

	// do abandon the current target? either reached or non-reachable anymore
	if ( abandon ) {
//		this->target_manager_ptr_->abandonTarget();
//		setState(actor::ACTOR_STATE_STOP_AND_STARE);
		return (false);
	}

	return (true);

}

SingleGoal::~SingleGoal() {
	// TODO Auto-generated destructor stub
}

} /* namespace core */
} /* namespace actor */
