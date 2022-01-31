#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_common/logger.h>
#include <string>

namespace hubero {

/**
 * @brief Class that acts as a localisation data interface between HuBeRo and the simulator software
 */
class LocalisationBase {
public:
	LocalisationBase(): initialized_(false) {}

	virtual void initialize(const std::string& world_frame_id) {
		frame_id_ = world_frame_id;
		initialized_ = true;
	}

	virtual void update(const Pose3& pose) {
		if (!isInitialized()) {
			HUBERO_LOG("[LocalisationBase] 'update' call could not be processed due to lack of initialization\r\n");
			return;
		}
		pose_ = pose;
	}

	/**
	 * @brief This version of update call must be used if simulator uses different coordinate frame compared to HuBeRo
	 * @details This must be called giving simulator version of @ref pose
	 * Usually it will be called from inside of the simulator plugin for localisation update
	 * @note Typically this is not necessary to be in use and only @ref update may be used since
	 * internally calls @ref update
	 */
	virtual void updateSimulator(const Pose3& pose) {
		update(pose);
	}

	virtual void update(
		const Pose3& pose,
		const Vector3& vel_lin,
		const Vector3& vel_ang,
		const Vector3& acc_lin,
		const Vector3& acc_ang
	) {
		if (!isInitialized()) {
			HUBERO_LOG("[LocalisationBase] 'update' call could not be processed due to lack of initialization\r\n");
			return;
		}
		pose_ = pose;
		vel_lin_ = vel_lin;
		vel_ang_ = vel_ang;
		acc_lin_ = acc_lin;
		acc_ang_ = acc_ang;
	}

	/**
	 * @brief Overloaded, extended version of updateSimulator method
	 */
	virtual void updateSimulator(
		const Pose3& pose,
		const Vector3& vel_lin,
		const Vector3& vel_ang,
		const Vector3& acc_lin,
		const Vector3& acc_ang
	) {
		update(pose, vel_lin, vel_ang, acc_lin, acc_ang);
	}

	inline bool isInitialized() const {
		return initialized_;
	}

	inline virtual std::string getFrame() const {
		return frame_id_;
	}

	inline virtual Pose3 getPose() const {
		return pose_;
	}

	/**
	 * @brief Additional method useful when character in a simulator uses strange coordinate system
	 * @details Normally, it does the same as @ref getPose
	 */
	inline virtual Pose3 getPoseSimulator() const {
		return pose_;
	}

	inline virtual Vector3 getVelocityLinear() const {
		return vel_lin_;
	}

	inline virtual Vector3 getVelocityAngular() const {
		return vel_ang_;
	}

	inline virtual Vector3 getAccelerationLinear() const {
		return acc_lin_;
	}

	inline virtual Vector3 getAccelerationAngular() const {
		return acc_ang_;
	}

protected:
	bool initialized_;
	std::string frame_id_;

	Pose3 pose_;
	Vector3 vel_lin_;
	Vector3 vel_ang_;
	Vector3 acc_lin_;
	Vector3 acc_ang_;
};

} // namespace hubero
