#include <hubero_gazebo/localisation_gazebo.h>
#include <hubero_common/logger.h>

namespace hubero {

LocalisationGazebo::LocalisationGazebo(): LocalisationBase::LocalisationBase() {}

void LocalisationGazebo::update(const Pose3& pose) {
	if (!isInitialized()) {
		HUBERO_LOG("[LocalisationGazebo] 'update' call could not be processed due to lack of initialization\r\n");
		return;
	}

    // some additional operations must be performed due to the ActorPlugins' human body frame
    // match local CS axes to the world frame
	Vector3 pos = pose.Pos();
	Vector3 rpy = pose.Rot().Euler();

    // prepare roll angle
	Angle roll(rpy.X() - IGN_PI_2);
	roll.Normalize();
	rpy.X() = roll.Radian();

    // pitch is straightforward
	rpy.Y() = 0.0;

    // prepare yaw angle
	Angle yaw(rpy.Z() - IGN_PI_2);
	yaw.Normalize();
	rpy.Z() = yaw.Radian();

    LocalisationBase::update(Pose3(pos, Quaternion(rpy)));
}

void LocalisationGazebo::update(
	const Pose3& pose,
	const Vector3& vel_lin,
	const Vector3& vel_ang,
	const Vector3& acc_lin,
	const Vector3& acc_ang
) {
	LocalisationBase::update(pose, vel_lin, vel_ang, acc_lin, acc_ang);
	// overwrite pose according to customized calculations (LocalisationGazebo::update)
	update(pose);
}

} // namespace hubero