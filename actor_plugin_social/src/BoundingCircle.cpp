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

ignition::math::Vector3d BoundingCircle::GetIntersection(ignition::math::Vector3d &_pt_dest) const {

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

BoundingCircle::~BoundingCircle() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace ActorUtils */
