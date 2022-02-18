#pragma once

#include <hubero_interfaces/localisation_base.h>
#include <hubero_common/time.h>

namespace hubero {

/**
 * @brief Gazebo-specific interface to provide localisation data from simulator to HuBeRo
 *
 * @details There are multiple problems with Actor Plugin. One of them is that relative pose of sensor links (that were
 * attached to human body) cannot be read from the plugin level. See a workaround in `bringup` package
 */
class LocalisationGazebo: public LocalisationBase {
public:
	/// Default constructor
	LocalisationGazebo();

	/**
	 * @brief Extended version of updateSimulator call that realizes workaround for Gazebo-specific issues
	 *
	 * Provides timestamp to compute velocity and acceleration.
	 *
	 * Since Gazebo does not store velocity or acceleration of the ActorPlugin, an extended version
	 * of updateSimulator must be used to provide valid velocity and acceleration
	 */
	void updateSimulator(const Pose3& pose, const Time& time);

	/**
	 * @brief Extended version of updateSimulator call that realizes workaround for Gazebo-specific issues
	 *
	 * For details see other @ref updateSimulator
	 */
	void updateSimulator(
		const Pose3& pose,
		const Vector3& vel_lin,
		const Vector3& vel_ang,
		const Vector3& acc_lin,
		const Vector3& acc_ang,
		const Time& time
	);

	/**
	 * @brief Retrieves the newest pose complement with simulator coordinate system
	 *
	 * The newest pose must be set by either @ref updateSimulator or @ref update
	 */
	virtual Pose3 getPoseSimulator() const override;

protected:
	/**
	 * @brief Computes linear and angular velocities and accelerations based on given pose and stored vel and acc
	 *
	 * This must be called before any change to member, e.g., @ref pose_, @ref vel_ang_ or @ref vel_lin_ occurs
	 *
	 * @param pose current pose expressed in the target frame (HuBeRo's)
	 * @param time current timestamp
	 */
	void computeVelocityAndAcceleration(Pose3 pose, Time time);

	/**
	 * @defgroup cvamembers Localisation state at the time of last velocity/acceleration computation
	 * @{
	 */
	Time time_cva_last_;
	Pose3 pose_cva_last_;
	Vector3 vel_lin_cva_last_;
	Vector3 vel_ang_cva_last_;

	/// @}
};

} // namespace hubero
