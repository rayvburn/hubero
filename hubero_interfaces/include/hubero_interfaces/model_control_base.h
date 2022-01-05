#pragma once

#include <hubero_common/logger.h>
#include <hubero_common/typedefs.h>
#include <string>
#include <functional>

namespace hubero {

/**
 * @brief Class that acts as a model control interface between simulator software and HuBeRo
 * @details Since each HuBeRo actor poses ModelControlBase he is able to directly control its 'dynamics' in the sim
 */
class ModelControlBase {
public:
	ModelControlBase(): initialized_(false) {}

	virtual void initialize(
		const std::string& frame_id,
		std::function<void(Pose3)> fun_pose,
		std::function<void(Vector3)> fun_ang_vel,
		std::function<void(Vector3)> fun_lin_vel,
		std::function<void(Vector3)> fun_ang_acc
		std::function<void(Vector3)> fun_lin_acc
	) {
		frame_id_ = frame_id;
		fun_pose_ = fun_pose;
		fun_ang_vel_ = fun_ang_vel;
		fun_lin_vel_ = fun_lin_vel;
		fun_ang_acc_ = fun_ang_acc;
		fun_lin_acc_ = fun_lin_acc;
		initialized_ = true;
	}

	virtual void update(
		const Pose3& pose,
		const Vector3& vel_ang,
		const Vector3& vel_lin,
		const Vector3& acc_ang,
		const Vector3& acc_lin
	) {
		if (!isInitialized()) {
			HUBERO_LOG("[ModelControlBase] 'update' call could not be processed due to lack of initialization\r\n");
			return;
		}

		if (fun_pose_ != nullptr) {
			fun_pose_(pose);
		} else {
			HUBERO_LOG("[ModelControlBase] Pose not updated since corresponding handler is nullptr\r\n");
		}

		if (fun_ang_vel_ != nullptr) {
			fun_ang_vel_(vel_ang);
		} else {
			HUBERO_LOG("[ModelControlBase] Angular velocity not updated since corresponding handler is nullptr\r\n");
		}

		if (fun_lin_vel_ != nullptr) {
			fun_lin_vel_(vel_lin);
		} else {
			HUBERO_LOG("[ModelControlBase] Linear velocity not updated since corresponding handler is nullptr\r\n");
		}

		if (fun_ang_acc_ != nullptr) {
			fun_ang_acc_(acc_ang);
		} else {
			HUBERO_LOG("[ModelControlBase] Angular accel not updated since corresponding handler is nullptr\r\n");
		}

		if (fun_lin_acc_ != nullptr) {
			fun_lin_acc_(acc_lin);
		} else {
			HUBERO_LOG("[ModelControlBase] Linear accel not updated since corresponding handler is nullptr\r\n");
		}
	}

	inline bool isInitialized() const {
		return initialized_;
	}

	inline std::string getFrame() const {
		return frame_id_;
	}

	// inline virtual void update(const int& model_name, const Pose3& pose, const Pose3& vel, const Pose3& acc) {}

protected:
	bool initialized_;

	/// Frame that pose is expressed in
	std::string frame_id_;

	std::function<void(Pose3)> fun_pose_;
	std::function<void(Vector3)> fun_ang_vel_;
	std::function<void(Vector3)> fun_lin_vel_;
	std::function<void(Vector3)> fun_ang_acc_;
	std::function<void(Vector3)> fun_lin_acc_;
};

} // namespace hubero
