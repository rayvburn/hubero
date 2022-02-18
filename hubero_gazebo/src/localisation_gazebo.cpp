#include <hubero_gazebo/localisation_gazebo.h>
#include <hubero_common/logger.h>

namespace hubero {

LocalisationGazebo::LocalisationGazebo(): LocalisationBase::LocalisationBase() {}

void LocalisationGazebo::updateSimulator(const Pose3& pose, const Time& time) {
	if (!isInitialized()) {
		HUBERO_LOG("[LocalisationGazebo] 'update' call could not be processed due to lack of initialization\r\n");
		return;
	}

	// some additional operations must be performed so the ActorPlugins' human body frame match local CS axes
	// to the world frame
	Vector3 pos = pose.Pos();
	Vector3 rpy = pose.Rot().Euler();

	// prepare roll angle
	Angle roll(rpy.X() - IGN_PI_2);
	roll.Normalize();
	rpy.X() = roll.Radian();

	// pitch '.Y()' does not require modifications

	// prepare yaw angle
	Angle yaw(rpy.Z() - IGN_PI_2);
	yaw.Normalize();
	rpy.Z() = yaw.Radian();

	auto pose_new = Pose3(pos, Quaternion(rpy));
	computeVelocityAndAcceleration(pose_new, time);
	LocalisationBase::update(pose_new);
}

void LocalisationGazebo::updateSimulator(
	const Pose3& pose,
	const Vector3& /*vel_lin*/,
	const Vector3& /*vel_ang*/,
	const Vector3& /*acc_lin*/,
	const Vector3& /*acc_ang*/,
	const Time& time
) {
	// will overwrite pose, velocities and accelerations
	updateSimulator(pose, time);
}

Pose3 LocalisationGazebo::getPoseSimulator() const {
	// need to switch back to ActorPlugin coordinate system (see @ref update for initial transformation)
	Vector3 rpy_gazebo = getPose().Rot().Euler();

	Angle roll(rpy_gazebo.X() + IGN_PI_2);
	roll.Normalize();
	rpy_gazebo.X() = roll.Radian();

	Angle yaw (rpy_gazebo.Z() + IGN_PI_2);
	yaw.Normalize();
	rpy_gazebo.Z() = yaw.Radian();

	return Pose3(getPose().Pos(), Quaternion(rpy_gazebo));
}

void LocalisationGazebo::computeVelocityAndAcceleration(Pose3 pose, Time time) {
	// time difference to compute velocity
	double time_diff = Time::computeDuration(time_cva_last_, time).getTime();

	// compute velocity (pose_ is previous pose (class member))
	vel_lin_.X((pose.Pos().X() - pose_cva_last_.Pos().X()) / time_diff);
	vel_lin_.Y((pose.Pos().Y() - pose_cva_last_.Pos().Y()) / time_diff);
	vel_lin_.Z((pose.Pos().Z() - pose_cva_last_.Pos().Z()) / time_diff);

	vel_ang_.X((pose.Rot().Roll() - pose_cva_last_.Rot().Roll()) / time_diff);
	vel_ang_.Y((pose.Rot().Pitch() - pose_cva_last_.Rot().Pitch()) / time_diff);
	vel_ang_.Z((pose.Rot().Yaw() - pose_cva_last_.Rot().Yaw()) / time_diff);

	// compute acceleration
	acc_lin_.X((vel_lin_.X() - vel_lin_cva_last_.X()) / time_diff);
	acc_lin_.Y((vel_lin_.Y() - vel_lin_cva_last_.Y()) / time_diff);
	acc_lin_.Z((vel_lin_.Z() - vel_lin_cva_last_.Z()) / time_diff);

	acc_ang_.X((vel_ang_.X() - vel_ang_cva_last_.X()) / time_diff);
	acc_ang_.Y((vel_ang_.Y() - vel_ang_cva_last_.Y()) / time_diff);
	acc_ang_.Z((vel_ang_.Z() - vel_ang_cva_last_.Z()) / time_diff);

	// store for the next computations
	time_cva_last_ = time;
	pose_cva_last_ = pose;
	vel_lin_cva_last_ = vel_lin_;
	vel_ang_cva_last_ = vel_ang_;
}

} // namespace hubero
