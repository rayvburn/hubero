#pragma once

#include <hubero_interfaces/localisation_base.h>

namespace hubero {

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

	Pose3 getPoseTransformed() const;
};

} // namespace hubero
