/*
 * Velocity.cpp
 *
 *  Created on: Jan 4, 2020
 *      Author: rayvburn
 */

#include <actor/core/Velocity.h>

namespace actor {
namespace core {

Velocity::Velocity() {
	// TODO Auto-generated constructor stub

}

void Velocity::configure(const std::shared_ptr<const ignition::math::Pose3d> pose_world_ptr,
			   const std::shared_ptr<const ignition::math::Pose3d> pose_world_prev_ptr)
{

	pose_world_ptr_ = pose_world_ptr;
	pose_world_prev_ptr_ = pose_world_prev_ptr;

}

void Velocity::calculate(const double &dt) {

	// =============== linear velocity

	ignition::math::Vector3d new_velocity;
	new_velocity.X( (pose_world_ptr_->Pos().X() - pose_world_prev_ptr_->Pos().X()) / dt );
	new_velocity.Y( (pose_world_ptr_->Pos().Y() - pose_world_prev_ptr_->Pos().Y()) / dt );
	new_velocity.Z( (pose_world_ptr_->Pos().Z() - pose_world_prev_ptr_->Pos().Z()) / dt );

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
	new_velocity.X( (pose_world_ptr_->Rot().Roll()  - pose_world_prev_ptr_->Rot().Roll()  ) / dt );
	new_velocity.Y( (pose_world_ptr_->Rot().Pitch() - pose_world_prev_ptr_->Rot().Pitch() ) / dt );
	new_velocity.Z( (pose_world_ptr_->Rot().Yaw()   - pose_world_prev_ptr_->Rot().Yaw()   ) / dt );

	// averaging block is not needed as angular velocity is not used by SFM
	velocity_ang_.X( new_velocity.X() );
	velocity_ang_.Y( new_velocity.Y() );
	velocity_ang_.Z( new_velocity.Z() );

}

ignition::math::Vector3d Velocity::getLinear() const {
	return (velocity_lin_);
}

ignition::math::Vector3d Velocity::getAngular() const {
	return (velocity_ang_);
}

Velocity::~Velocity() {
	// TODO Auto-generated destructor stub
}

} /* namespace core */
} /* namespace actor */
