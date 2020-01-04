/*
 * Velocity.h
 *
 *  Created on: Jan 4, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_VELOCITY_H_
#define INCLUDE_ACTOR_CORE_VELOCITY_H_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <memory>

namespace actor {
namespace core {

class Velocity {

public:

	Velocity();

	void configure(const std::shared_ptr<const ignition::math::Pose3d> pose_world_ptr_,
				   const std::shared_ptr<const ignition::math::Pose3d> pose_world_prev_ptr_);
	void calculate(const double &dt);

	ignition::math::Vector3d getLinear() const;
	ignition::math::Vector3d getAngular() const;

	virtual ~Velocity();

private:

    /// \brief Actual linear velocity_ of the actor
    ignition::math::Vector3d velocity_lin_;

    /// \brief Actual linear velocity_ of the actor
    ignition::math::Vector3d velocity_ang_;

    /// \brief An array that stores last 50 velocities -
    /// average velocity is calculated to provide smoother SFM operation
    std::array<ignition::math::Vector3d, 50> velocities_lin_to_avg_;

    /// \brief Actor's world pose - note: the coordinate system of an actor
	/// is rotated 90 deg CCW around world coordinate system's Z axis
	const std::shared_ptr<const ignition::math::Pose3d> pose_world_ptr_;

    /// \brief Previous actor_ptr_'s pose
    const std::shared_ptr<const ignition::math::Pose3d> pose_world_prev_ptr_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_VELOCITY_H_ */
