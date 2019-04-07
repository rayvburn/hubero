/*
 * Circle.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef SRC_INFLATION_CIRCLE_H_
#define SRC_INFLATION_CIRCLE_H_


#include <ignition/math/Vector3.hh>
#include <string>
#include <visualization_msgs/Marker.h>
#include <tuple>

namespace actor {
namespace inflation {

class Circle {

public:

	Circle();

	void setRadius(const double &radius);
	void setCenter(const ignition::math::Vector3d &center_point);

	std::tuple<bool, ignition::math::Vector3d> getIntersection(const ignition::math::Vector3d &pt_dest) const;
	double getRadius() const;
	ignition::math::Vector3d getCenter() const;
	bool doesContain(const ignition::math::Vector3d &pt_dest) const;
	visualization_msgs::Marker getMarkerConversion() const;

	virtual ~Circle();

private:

	double radius_;
	ignition::math::Vector3d center_;
};

} /* namespace inflation */
} /* namespace actor */

#endif /* SRC_INFLATION_CIRCLE_H_ */
