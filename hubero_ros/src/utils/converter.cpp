#include <hubero_ros/utils/converter.h>

namespace hubero {

geometry_msgs::Pose ignPoseToMsgPose(const Pose3& pose) {
	geometry_msgs::Pose geom_pose;
	geom_pose.position.x = pose.Pos().X();
	geom_pose.position.y = pose.Pos().Y();
	geom_pose.position.z = pose.Pos().Z();
	geom_pose.orientation.x = pose.Rot().X();
	geom_pose.orientation.y = pose.Rot().Y();
	geom_pose.orientation.z = pose.Rot().Z();
	geom_pose.orientation.w = pose.Rot().W();
	return geom_pose;
}

geometry_msgs::Transform ignPoseToMsgTf(const Pose3& pose) {
	geometry_msgs::Transform tf_stamp;
	tf_stamp.translation.x = pose.Pos().X();
	tf_stamp.translation.y = pose.Pos().Y();
	tf_stamp.translation.z = pose.Pos().Z();

	tf_stamp.rotation.x = pose.Rot().X();
	tf_stamp.rotation.y = pose.Rot().Y();
	tf_stamp.rotation.z = pose.Rot().Z();
	tf_stamp.rotation.w = pose.Rot().W();
	return tf_stamp;
}

geometry_msgs::Twist ignVectorsToMsgTwist(const Vector3& vel_lin, const Vector3& vel_ang) {
	geometry_msgs::Twist twist;
	twist.linear.x = vel_lin.X();
	twist.linear.y = vel_lin.Y();
	twist.linear.z = vel_lin.Z();

	twist.angular.x = vel_ang.X();
	twist.angular.y = vel_ang.Y();
	twist.angular.z = vel_ang.Z();
	return twist;
}

Vector3 msgTwistToIgnVector(const geometry_msgs::Twist& twist) {
	return Vector3(twist.linear.x, twist.linear.y, twist.angular.z);
}

} // namespace hubero
