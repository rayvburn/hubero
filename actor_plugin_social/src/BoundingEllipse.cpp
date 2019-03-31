/*
 * BoundingEllipse.cpp
 *
 *  Created on: Mar 31, 2019
 *      Author: rayvburn
 */

#include "BoundingEllipse.h"
#include <ignition/math/Angle.hh>
#include <ignition/math/Quaternion.hh>

namespace ActorUtils {

// ------------------------------------------------------------------- //

BoundingEllipse::BoundingEllipse(): a_major_(1.20), b_minor_(0.50), yaw_(0.00) { }

// ------------------------------------------------------------------- //

BoundingEllipse::BoundingEllipse(const double &a_major, const double &b_minor, const double &yaw,
								 const ignition::math::Vector3d &center_point,
								 const ignition::math::Vector3d &offset_vector) {

	init(a_major, b_minor, yaw, center_point, offset_vector);
}

// ------------------------------------------------------------------- //

void BoundingEllipse::init(const double &a_major, const double &b_minor, const double &yaw,
		  const ignition::math::Vector3d &center_point,
		  const ignition::math::Vector3d &offset_vector) {

	a_major_ = a_major;
	b_minor_ = b_minor;
	yaw_ = yaw;
	center_ = center_point;
	offset_ = offset_vector;
}

// ------------------------------------------------------------------- //

void BoundingEllipse::setSemiMajorAxis(const double &a_major) {
	a_major_ = a_major;
}

// ------------------------------------------------------------------- //

void BoundingEllipse::setSemiMinorAxis(const double &b_minor) {
	b_minor_ = b_minor;
}

// ------------------------------------------------------------------- //

void BoundingEllipse::setYaw(const double &yaw_ellipse) {
	yaw_ = yaw_ellipse;
}

// ------------------------------------------------------------------- //

void BoundingEllipse::setCenter(const ignition::math::Vector3d &center_point) {
	center_ = center_point;
}

// ------------------------------------------------------------------- //

void BoundingEllipse::setCenterOffset(const ignition::math::Vector3d &offset_vector) {
	offset_ = offset_vector;
}

// ------------------------------------------------------------------- //

void BoundingEllipse::updatePose(const ignition::math::Pose3d &pose) {
	center_ = pose.Pos();
	yaw_ = pose.Rot().Yaw() - IGN_PI_2; // coordinate systems transformation
}

// ------------------------------------------------------------------- //

/// ellipse orientation must match world's coordinate system -
/// a conversion from actor's coordinate system must be done!
ignition::math::Vector3d BoundingEllipse::getIntersection(const ignition::math::Vector3d &pt_dest) const {

//	/*
//	 * https://en.wikipedia.org/wiki/Ellipse#Shifted_Ellipse
//	 *
//	 * Using the sine and cosine functions a parametric representation
//	 * of the ellipse with shifted center is as follows:
//	 *
//	 * (c1 + a*cos(theta); c2 + b*sin(theta),
//	 * where 0 <= theta < 2pi
//	 *
//	 */
//
//	// 1st calculate the angle from center to the dest point
//	ignition::math::Angle theta;
//	ignition::math::Vector3d to_dest;
//	to_dest = pt_dest - (center_ + offset_);
//	theta.Radian(std::atan2(to_dest.Y(), to_dest.X()));
//	theta.Normalize();
//
//	// 2nd take the ellipse orientation into consideration
//	theta.Radian( theta.Radian() + yaw_ );
//	theta.Normalize();
//
//	// 3rd find the point of intersection of the to_dest vector and the ellipse
//	ignition::math::Vector3d pt_intersection;
//	pt_intersection.X( center_.X() + ( offset_.X() + (a_major_ * cos(theta.Radian())) ) );
//	pt_intersection.Y( center_.Y() + ( offset_.Y() + (b_minor_ * sin(theta.Radian())) ) );
//	pt_intersection.Z(0.0f);
//
//	return (pt_intersection);

	ignition::math::Vector3d pt_intersection;
	double angle_center_dest;
	std::tie(pt_intersection, angle_center_dest) = getIntersectionExt(pt_dest);
	return (pt_intersection);

}

// ------------------------------------------------------------------- //

bool BoundingEllipse::isWithin(const ignition::math::Vector3d &pt) const {

	ignition::math::Vector3d pt_to_check;
	double angle_center_dest;
	std::tie(pt_to_check, angle_center_dest) = getIntersectionExt(pt);

	// pt_to_check length of (from center)
	pt_to_check.Z(0.0);

	ignition::math::Vector3d pt_on_ellipse;
	pt_on_ellipse.X( center_.X() + ( offset_.X() + (a_major_ * cos(angle_center_dest)) ) );
	pt_on_ellipse.Y( center_.Y() + ( offset_.Y() + (b_minor_ * sin(angle_center_dest)) ) );
	pt_on_ellipse.Z(0.0f);

	if ( (pt_to_check - (center_ + offset_)).Length() <= (pt_on_ellipse - (center_ + offset_)).Length()  ) {
		return (true);
	}

	return (false);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d BoundingEllipse::getCenter() const {
	return (center_);
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d BoundingEllipse::getCenterOffsets() const {
	return (offset_);
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker BoundingEllipse::getMarkerConversion() const {

	visualization_msgs::Marker marker;

	marker.header.frame_id = "map";
	//marker.header.stamp = 0.0;
	marker.ns = "test"; // TODO: change ns name
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::MODIFY;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = (center_ + offset_).X();
	marker.pose.position.y = (center_ + offset_).Y();
	marker.pose.position.z = 0.9; // half of a typical person height

	// convert rpy to quaternion and set marker's orientation
	ignition::math::Quaterniond rot(0.0, 0.0, yaw_);
	marker.pose.orientation.x = rot.X();
	marker.pose.orientation.y = rot.Y();
	marker.pose.orientation.z = rot.Z();
	marker.pose.orientation.w = rot.W();

	// scale
	marker.scale.x = a_major_;
	marker.scale.y = b_minor_;
	marker.scale.z = 1.8f; 	// typical person height

	marker.color.a = 0.3; 	// alpha channel
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	return (marker);

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Vector3d, double> BoundingEllipse::getIntersectionExt(const ignition::math::Vector3d &pt_dest)
	const {

	/*
	 * https://en.wikipedia.org/wiki/Ellipse#Shifted_Ellipse
	 *
	 * Using the sine and cosine functions a parametric representation
	 * of the ellipse with shifted center is as follows:
	 *
	 * (c1 + a*cos(theta); c2 + b*sin(theta),
	 * where 0 <= theta < 2pi
	 *
	 */

	// 1st calculate the angle from center to the dest point
	ignition::math::Angle theta;
	ignition::math::Vector3d to_dest;
	to_dest = pt_dest - (center_ + offset_);
	theta.Radian(std::atan2(to_dest.Y(), to_dest.X()));
	theta.Normalize();

	// 2nd take the ellipse orientation into consideration
	theta.Radian( theta.Radian() + yaw_ );
	theta.Normalize();

	// 3rd find the point of intersection of the to_dest vector and the ellipse
	ignition::math::Vector3d pt_intersection;
	pt_intersection.X( center_.X() + ( offset_.X() + (a_major_ * cos(theta.Radian())) ) );
	pt_intersection.Y( center_.Y() + ( offset_.Y() + (b_minor_ * sin(theta.Radian())) ) );
	pt_intersection.Z(0.0f);

	return (std::make_tuple( pt_intersection, theta.Radian() ));

}

// ------------------------------------------------------------------- //

BoundingEllipse::~BoundingEllipse() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ActorUtils */
