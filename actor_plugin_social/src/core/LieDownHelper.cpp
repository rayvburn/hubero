/*
 * LieDownHelper.cpp
 *
 *  Created on: Oct 9, 2019
 *      Author: rayvburn
 */

#include <core/LieDownHelper.h>
#include <ignition/math/Angle.hh>

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

LieDownHelper::LieDownHelper(): rotation_(0.0), pose_lie_height_(0.0),
		is_lying_(false), do_finish_(false) {}

// ------------------------------------------------------------------- //

void LieDownHelper::setLyingHeight(const double &height) {
	pose_lie_height_ = height;
}

// ------------------------------------------------------------------- //

void LieDownHelper::setLyingPose(const ignition::math::Pose3d &pose_center) {
	pose_center_ = pose_center;
}

// ------------------------------------------------------------------- //

void LieDownHelper::setRotation(const double &rot) {
	rotation_ = rot;
}

// ------------------------------------------------------------------- //

void LieDownHelper::setPoseBeforeLying(const ignition::math::Pose3d &pose) {
	pose_lie_start_ = pose;
}

// ------------------------------------------------------------------- //

void LieDownHelper::setLying(const bool &lied_down) {
	is_lying_ = lied_down;
	do_finish_ = false; // either started lying or whole lying procedure stopped
}

// ------------------------------------------------------------------- //

void LieDownHelper::stopLying() {
	do_finish_ = true;
}

// ------------------------------------------------------------------- //

ignition::math::Pose3d LieDownHelper::computePoseFinishedLying() const {

	ignition::math::Angle yaw_new = pose_lie_start_.Rot().Yaw();
	yaw_new += ignition::math::Angle::HalfPi;
	yaw_new.Normalize();

	ignition::math::Pose3d new_pose = pose_lie_start_;

	// update orientation component of the pose
	ignition::math::Quaterniond quat;
	quat.Euler(new_pose.Rot().Roll(), new_pose.Rot().Pitch(), yaw_new.Radian());

	// Rot is a reference
	new_pose.Rot() = quat;

	return (new_pose);

}

// ------------------------------------------------------------------- //

bool LieDownHelper::isLyingDown() const {
	return (is_lying_);
}

// ------------------------------------------------------------------- //

double LieDownHelper::getLyingHeight() const {
	return (pose_lie_height_);
}

// ------------------------------------------------------------------- //

ignition::math::Pose3d LieDownHelper::getPoseBeforeLying() const {
	return (pose_lie_start_);
}

// ------------------------------------------------------------------- //

ignition::math::Pose3d LieDownHelper::getPoseLying() const {

	// NOTE: object's yaw affect the final lying orientation
	ignition::math::Angle yaw_fake(pose_center_.Rot().Yaw());
	yaw_fake.Radian(yaw_fake.Radian() + rotation_);
	yaw_fake.Normalize();

	ignition::math::Pose3d pose_center_shifted = pose_center_;
	pose_center_shifted.Pos().Z() = pose_lie_height_;

	// update orientation component of the pose
	ignition::math::Quaterniond quat;
	quat.Euler(pose_center_.Rot().Roll(), pose_center_.Rot().Pitch(), yaw_fake.Radian());

	// Rot is a reference
	pose_center_shifted.Rot() = quat;

	return (pose_center_shifted);

}

// ------------------------------------------------------------------- //

bool LieDownHelper::doStopLying() const {
	return (do_finish_);
}

// ------------------------------------------------------------------- //

LieDownHelper::~LieDownHelper() {}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
