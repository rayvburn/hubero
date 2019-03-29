/*
 * BoundingCircle.cpp
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#include <BoundingCircle.h>
#include <ignition/math/Angle.hh>
#include <cmath> // atan2()

namespace ActorUtils {

// ------------------------------------------------------------------- //

BoundingCircle::BoundingCircle():
	radius(0.3),
	center(0.0, 0.0, 0.0)
{
}

// ------------------------------------------------------------------- //

void BoundingCircle::SetRadius(const double &_radius) {
	this->radius = _radius;
}

// ------------------------------------------------------------------- //

void BoundingCircle::SetCenter(const ignition::math::Vector3d &_center_point) {
	this->center = _center_point;
}

// ------------------------------------------------------------------- //

/*
 * GetIntersection takes a destination point and calculates the intersection point on the circle which
 * creates a line from circle's center (actor's center) to destination point while passing through the circle
 */
ignition::math::Vector3d BoundingCircle::GetIntersection(const ignition::math::Vector3d &_pt_dest) const {

	/*
	 * circle equation in parametric form:
	 * x = a + r*cos(theta)
	 * y = b + r*sin(theta)
	 *
	 * where:
	 * 	o 	(a,b) are center coordinates
	 * 	o	(x,y) set of points creating the circle
	 *
	 */

	// 1st calculate the angle from center to dest point
	ignition::math::Angle theta;
	ignition::math::Vector3d dest_center;
	dest_center = _pt_dest - center;
	theta.Radian(std::atan2(dest_center.Y(), dest_center.X()));
	theta.Normalize();

	// 2nd find the point of intersection of the dest_center vector and the circle
	ignition::math::Vector3d pt_intersection;
	pt_intersection.X( this->center.X() + this->radius * cos(theta.Radian()) );
	pt_intersection.Y( this->center.Y() + this->radius * sin(theta.Radian()) );
	pt_intersection.Z(0.0f);

	return (pt_intersection);

}

// ------------------------------------------------------------------- //

double BoundingCircle::GetRadius() const {
	return (this->radius);
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker BoundingCircle::GetMarkerConversion() const {

	visualization_msgs::Marker marker;

	marker.header.frame_id = "map";
	//marker.header.stamp = 0.0;
	marker.ns = "test";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::MODIFY;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = this->center.X();
	marker.pose.position.y = this->center.Y();
	marker.pose.position.z = 0.9; // half of a typical person height

	// cylinder - yaw orientation doesn't matter
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// scale
	marker.scale.x = this->radius * 2.0;
	marker.scale.y = this->radius * 2.0;
	marker.scale.z = 1.8f; // typical person height

	marker.color.a = 0.2; // alpha channel
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	return (marker);
}

// ------------------------------------------------------------------- //

BoundingCircle::~BoundingCircle() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ActorUtils */
