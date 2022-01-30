#pragma once

#include <hubero_interfaces/localisation_base.h>

namespace hubero {

/**
 * @brief Gazebo-specific interface to provide localisation data from simulator to HuBeRo
 *
 * @details There are multiple problems with Actor Plugin. One of them is that relative pose of sensor links (that were
 * attached to human body) cannot be read from the plugin level. See a workaround in `bringup` package
 */
class LocalisationGazebo: public LocalisationBase {
public:
    LocalisationGazebo();

    virtual void update(const Pose3& pose) override;

    virtual void update(
		const Pose3& pose,
		const Vector3& vel_lin,
		const Vector3& vel_ang,
		const Vector3& acc_lin,
		const Vector3& acc_ang
	) override;

	virtual Pose3 getPoseSimulator() const override;
};

} // namespace hubero
