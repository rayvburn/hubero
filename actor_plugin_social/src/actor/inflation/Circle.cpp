/*
 * Circle.cpp
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#include <actor/inflation/Circle.h>
#include <ignition/math/Angle.hh>
#include <cmath> // atan2()
#include <string>
#include "actor/FrameGlobal.h" // global frame id

namespace actor {
namespace inflation {

// ------------------------------------------------------------------- //

Circle::Circle():
	radius_(0.3),
	center_(0.0, 0.0, 0.0)
{
	this->type_ = BORDER_CIRCLE;
}

// ------------------------------------------------------------------- //

void Circle::setRadius(const double &radius) {
	radius_ = radius;
}

// ------------------------------------------------------------------- //

void Circle::updatePose(const ignition::math::Pose3d &pose) {
	center_ = pose.Pos();
}

// ------------------------------------------------------------------- //

/*
 * GetIntersection takes a destination point and calculates the intersection point on the circle which
 * creates a line from circle's center (actor's center) to destination point while passing through the circle
 */
std::tuple<bool, ignition::math::Vector3d> Circle::doesIntersect(const ignition::math::Vector3d &pt_dest) const {

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

	// 1st calculate the angle from center to the dest point
	ignition::math::Angle theta;
	ignition::math::Vector3d dest_center;
	dest_center = pt_dest - center_;
	theta.Radian(std::atan2(dest_center.Y(), dest_center.X()));
	theta.Normalize();

	// 2nd find the point of intersection of the dest_center vector and the circle
	ignition::math::Vector3d pt_intersection;
	pt_intersection.X( center_.X() + radius_ * cos(theta.Radian()) );
	pt_intersection.Y( center_.Y() + radius_ * sin(theta.Radian()) );
	pt_intersection.Z(0.0f);

	// intersection with circle is always found - tuple here just for equality
	// with Ellipse class' method
	return ( std::make_tuple(true, pt_intersection) );

}

// ------------------------------------------------------------------- //

double Circle::getRadius() const {
	return (radius_);
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Circle::getCenter() const {
	return (center_);
}

// ------------------------------------------------------------------- //

bool Circle::doesContain(const ignition::math::Vector3d &pt) const {

	ignition::math::Vector3d dist_to_compare;
	dist_to_compare = pt - center_;
	dist_to_compare.Z(0.0); // 3d vector comparison with planar radius

	if ( dist_to_compare.Length() <= radius_ ) {
		return (true);
	}

	return (false);
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Circle::getMarkerConversion() const {

	visualization_msgs::Marker marker;

	marker.header.frame_id = actor::FrameGlobal::getFrame();
	//marker.header.stamp = 0.0;
	marker.ns = "test"; // TODO: change ns name
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::MODIFY;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = center_.X();
	marker.pose.position.y = center_.Y();
	marker.pose.position.z = 0.9; // half of a typical person height

	// cylinder - yaw orientation doesn't matter
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// scale
	marker.scale.x = radius_ * 2.0;
	marker.scale.y = radius_ * 2.0;
	marker.scale.z = 1.8f; // typical person height

	marker.color.a = 0.5; // alpha channel
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	return (marker);
}

// ------------------------------------------------------------------- //

Circle::~Circle() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace inflation */
} /* namespace actor */
