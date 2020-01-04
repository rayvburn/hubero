/*
 * Move.cpp
 *
 *  Created on: Jan 3, 2020
 *      Author: rayvburn
 */

#include <actor/core/Move.h>

namespace actor {
namespace core {

Move::Move() {
	// TODO Auto-generated constructor stub

}

void Move::configure(std::shared_ptr<Path> path_storage_ptr) {
	path_storage_ptr_ = path_storage_ptr;
}
void Move::configure(std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr) {
	social_conductor_.configure(params_ptr->getBehaviourParams());
//	social_conductor_.configure(params_ptr_->getBehaviourParams());
}
void Move::configure(std::shared_ptr<sfm::SocialForceModel> sfm_ptr) {
	sfm_ptr_ = sfm_ptr;
}
void Move::configure(std::shared_ptr<const actor::core::CommonInfo> common_info_ptr) {
	common_info_ptr_ = common_info_ptr;
}
void Move::configure(std::shared_ptr<const std::vector<std::string> >  ignored_models_ptr) {
	ignored_models_ptr_ = ignored_models_ptr;
}
void Move::configure(gazebo::physics::WorldPtr world_ptr) {
	world_ptr_ = world_ptr;
}
void Move::configure(std::shared_ptr<const actor::core::Velocity> velocity_ptr) {
	velocity_ptr_ = velocity_ptr;
}


void Move::execute(const double &dt) {

	// calculate `social` force (i.e. `internal` and `interaction` components)
	sfm_ptr_->computeSocialForce(world_ptr_, target_manager_ptr_->getPose(), velocity_ptr_->getLinear(),
						    target_manager_ptr_->getCheckpoint(),
						    *common_info_ptr_.get(), dt, *ignored_models_ptr_);

	// actual `social` vector
	ignition::math::Vector3d human_action_force(0.0, 0.0, 0.0);

	// evaluate whether more complex forces are supposed to be calculated
	if ( !params_ptr_->getSfmParams().disable_interaction_forces ) {

		// execute fuzzy operations block
		fuzzy_processor_.load(sfm_ptr_->getDirectionAlpha(), sfm_ptr_->getDirectionBetaDynamic(),
							  sfm_ptr_->getRelativeLocationDynamic(), sfm_ptr_->getDistanceAngleDynamic());
		fuzzy_processor_.process();

		// create a force vector according to the activated `social behaviour`
		social_conductor_.apply(sfm_ptr_->getForceCombined(), sfm_ptr_->getDirectionAlpha(), sfm_ptr_->getDistanceDynamic(),
								fuzzy_processor_.getOutput());

		// assign `social` vector
		human_action_force = social_conductor_.getSocialVector();

	}

    // according to the force, calculate a new pose
	ignition::math::Pose3d new_pose = sfm_ptr_->computeNewPose(target_manager_ptr_->getPose(), velocity_ptr_->getLinear(),
														  sfm_ptr_->getForceCombined() + human_action_force,
														  target_manager_ptr_->getCheckpoint(), dt);

	// object info update
	dist_traveled_ = (new_pose.Pos() - target_manager_ptr_->getPose().Pos()).Length();

	// update the local copy of the actor's pose
	target_manager_ptr_->getPose() = new_pose;

	// collect data to visualize actor's path
	path_storage_ptr_->collect(target_manager_ptr_->getPose().Pos(), sfm_ptr_->getDistanceClosestStaticObstacle());

}

ignition::math::Vector3d Move::getSocialVector() const {
	return (social_conductor_.getSocialVector());
}

std::string Move::getBehaviourActive() const {
	return (social_conductor_.getBehaviourActive());
}

Move::~Move() {
	// TODO Auto-generated destructor stub
}

} /* namespace core */
} /* namespace actor */
