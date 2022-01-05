#pragma once

#include <hubero_common/typedefs.h>
#include <string>

namespace converter {
	/// @brief Retrieves a single orientation component - rotation around Z axis
	/// @note Putting this method into .cpp causes linking error in `hubero_local_planner`'s library
	/// of the same name
	inline double yawFromIgnPose(const Pose3& pose) {
		// the disparity in names seems to come from the Euler angles order (YPR)?
		return pose.Rot().Roll();
	}
}; // namespace converter
