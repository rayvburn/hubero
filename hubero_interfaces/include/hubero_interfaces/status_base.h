#pragma once

#include <hubero_common/typedefs.h>
#include <string>

namespace hubero {

/**
 * @brief Creates interface class for HuBeRo Actor status
 *
 * This class aims to be extended by specific framework, so Actor data may be broadcasted
 */
class StatusBase {
public:
	StatusBase():
		actor_name_("actor_name_undefined"),
		frame_pose_(""),
		initialized_(false) {}

	/**
	 * @brief Initializes class basic members
	 *
	 * @param actor_name name of the actor (owner)
	 * @param frame_name name of the frame that poses and velocities will be expressed in
	 */
	virtual void initialize(const std::string& actor_name, const std::string& frame_name) {
		actor_name_ = actor_name;
		frame_pose_ = frame_name;
		initialized_ = true;
	}

	/**
	 * @brief Allows to process update of pose and velocities
	 *
	 * This is a base class so does nothing, but extensions will likely broadcast these data further
	 */
	virtual void update(const Pose3& pose, const Vector3& vel_lin, const Vector3& vel_ang) {}

	/**
	 * @brief Returns true if class was initialized successfully
	 */
	inline bool isInitialized() const {
		return initialized_;
	}

protected:
	std::string actor_name_;
	std::string frame_pose_;
	bool initialized_;

}; // class StatusBase

} // namespace hubero

