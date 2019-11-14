/*
 * Circle.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef SRC_INFLATION_CIRCLE_H_
#define SRC_INFLATION_CIRCLE_H_


#include <ignition/math/Vector3.hh>
#include <visualization_msgs/Marker.h>
#include <tuple>

namespace actor {
namespace inflation {

/// \brief A class which creates a circular
/// `inflation` figure around the actor;
/// used by SFM
class Circle {

public:

	/// \brief Default constructor
	Circle();

	/// \brief Sets the circle's radius to a given value
	void setRadius(const double &radius);

	/// \brief Moves the circle's center to a given point
	void setCenter(const ignition::math::Vector3d &center_point);

	/// \brief Method which checks whether a circle
	/// contains a given point; checks if the point
	/// lays within circle's bounds
	bool doesContain(const ignition::math::Vector3d &pt_dest) const;

	/// \brief Method which finds an intersection point
	/// based on the circle's center and a given point
	/// \param[in] pt_dest is a destination point;
	/// the direction from circle's center to a destination
	/// point is calculated and based on direction angle
	/// an intersection point with the circle's bound is found
	/// \return A tuple consisting of bool flag (always true),
	/// a point coordinates in which a line of direction equal
	/// to the angle intersects the circle box; one solution
	/// available only
	std::tuple<bool, ignition::math::Vector3d> getIntersection(const ignition::math::Vector3d &pt_dest) const;
	// FIXME: it should be named `doesIntersect`?

	/// \brief Returns the circle's radius
	double getRadius() const;

	/// \brief Returns the circle's center point
	ignition::math::Vector3d getCenter() const;

	/// \brief Returns a visualization_msgs::Marker
	/// instance which is created by circle conversion;
	/// the height of a circle is equal to 1.8 m
	visualization_msgs::Marker getMarkerConversion() const;

	/// \brief Default destructor
	virtual ~Circle();

private:

	/// \brief Stores a radius of a circle
	double radius_;

	/// \brief Stores a center point of the circle
	ignition::math::Vector3d center_;
};

} /* namespace inflation */
} /* namespace actor */

#endif /* SRC_INFLATION_CIRCLE_H_ */
