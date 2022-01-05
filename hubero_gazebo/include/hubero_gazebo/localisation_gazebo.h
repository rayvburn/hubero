#pragma once

#include <hubero_interface/localisation_base.h>

namespace hubero {

class LocalisationGazebo: public LocalisationBase {
    LocalisationGazebo(const std::string& world_frame_id);

    virtual void update(const Pose3& pose) override;

    virtual void update(
		const Pose3& pose,
		const Vector3& vel_lin,
		const Vector3& vel_ang,
		const Vector3& acc_lin,
		const Vector3& acc_ang
	) override;
};

} // namespace hubero
