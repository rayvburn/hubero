#pragma once

#include <string>
#include <hubero_common/typedefs.h>

namespace hubero {

class ModelGeometry {
public:
	ModelGeometry(
		const std::string& name = std::string(),
		const std::string& frame_id = std::string(),
		const Pose3& pose = Pose3(),
		const Vector3& vel_lin = Vector3(),
		const Vector3& vel_ang = Vector3(),
		const Vector3& acc_lin = Vector3(),
		const Vector3& acc_ang = Vector3(),
		const BBox& box = BBox()
	) {
		name_ = name;
		frame_id_ = frame_id;
		pose_ = pose;
		vel_lin_ = vel_lin;
		vel_ang_ = vel_ang;
		acc_lin_ = acc_lin;
		acc_ang_ = acc_ang;
		box_ = box;
	}

	std::string getName() const {
		return name_;
	}

	std::string getFrameId() const {
		return frame_id_;
	}

	Pose3 getPose() const {
		return pose_;
	}

	Vector3 getVelocityLinear() const {
		return vel_lin_;
	}

	Vector3 getVelocityAngular() const {
		return vel_ang_;
	}

	Vector3 getAccelerationLinear() const {
		return acc_lin_;
	}

	Vector3 getAccelerationAngular() const {
		return acc_ang_;
	}

	BBox getBoundingBox() const {
		return box_;
	}

protected:
	std::string name_;
	std::string frame_id_;
	Pose3 pose_;
	Vector3 vel_lin_;
	Vector3 vel_ang_;
	Vector3 acc_lin_;
	Vector3 acc_ang_;
	BBox box_;
};

} // namespace hubero