/*
 * Box.h
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#ifndef SRC_INFLATION_BOX_H_
#define SRC_INFLATION_BOX_H_

#include <actor/inflation/Border.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Box.hh>
#include <ignition/math/Line3.hh>
#include <tuple>

namespace actor {
namespace inflation {

/* This is a WIP - `roll` and `pitch` rotations are not supported, only the `yaw` angle is */

/// \brief A wrapper for a ignition::math::Box class;
/// creates an `inflation` figure around the actor
/// used by SFM
class Box : public Border {

public:

	/// \brief Default constructor
	Box();

	/// \brief ignition's Box copy constructor
	Box(const ignition::math::Box &bb);

	/// \brief Method which initializes a box
	/// with a given lengths of x, y  and z
	/// sides
	void init(const double &x_half_len, const double &y_half_len, const double &z_half_len);

	/// \brief Method which copies a given
	/// ignition's Box instance
	void setBox(const ignition::math::Box &bb);

	/// \brief Method which updates a class'es
	/// internal Box instance's pose
	virtual void updatePose(const ignition::math::Pose3d &new_pose) override;

	/// \brief Method which checks whether a box
	/// contains a given point; checks if the point
	/// lays within box'es bounds
	virtual bool doesContain(const ignition::math::Vector3d &pt) const override;

	/// \brief Method which checks whether a given
	/// line does intersect the box;
	/// \return A tuple consisting of bool flag which
	/// is true when line intersects a box and a point
	/// coordinates in which a line intersects a box;
	/// it is assumed that a line does not have
	/// 2 intersection points
	virtual std::tuple<bool, ignition::math::Vector3d> doesIntersect(const ignition::math::Line3d &line) const override;

	/// \brief Returns a visualization_msgs::Marker
	/// instance which is created by Box conversion
	virtual visualization_msgs::Marker getMarkerConversion() const override;

	/// \brief Returns a Box'es center coordinates
	virtual ignition::math::Vector3d getCenter() const override;

	/// \brief Returns a ignition's Box
	/// instance which class wraps
	virtual ignition::math::Box getBox() const override;

	// TODO: get rid of these 2 below (either delete completely and modify code or introduce double inheritance :Box)
	/// \brief Returns a Box'es minimum
	/// vector coordinates
	ignition::math::Vector3d getMin() const;

	/// \brief Returns a Box'es maximum
	/// vector coordinates
	ignition::math::Vector3d getMax() const;

	/// \brief Default destructor
	virtual ~Box();

	virtual void test() override { std::cout << "BOX CLASS" << std::endl; };

private:

	/// \brief An ignition's Box instance which
	/// a class wraps
	ignition::math::Box bb_;

	/// \brief A half length of a Box'es X-side
	double bb_x_half_;

	/// \brief A half length of a Box'es Y-side
	double bb_y_half_;

	/// \brief A half length of a Box'es Z-side
	double bb_z_half_;

};

} /* namespace inflation */
} /* namespace actor */

#endif /* SRC_INFLATION_BOX_H_ */
