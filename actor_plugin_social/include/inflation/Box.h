/*
 * Box.h
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#ifndef SRC_INFLATION_BOX_H_
#define SRC_INFLATION_BOX_H_


#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Box.hh>
#include <ignition/math/Line3.hh>
#include <tuple>

namespace actor {
namespace inflation {

class Box {

public:

	Box();
	Box(const ignition::math::Box &bb);
	void init(const double &x_half_len, const double &y_half_len, const double &z_half_len);
	void updatePose(const ignition::math::Pose3d &new_pose);
	bool doesContain(const ignition::math::Vector3d &pt) const;
	std::tuple<bool, ignition::math::Vector3d> doesIntersect(const ignition::math::Line3d &line) const;
	ignition::math::Vector3d getCenter() const;
	ignition::math::Vector3d getMin() const;
	ignition::math::Vector3d getMax() const;
	ignition::math::Box getBox() const;
	virtual ~Box();

private:

	ignition::math::Box bb_;
	double bb_x_half_;
	double bb_y_half_;
	double bb_z_half_;

};

} /* namespace inflation */
} /* namespace actor */

#endif /* SRC_INFLATION_BOX_H_ */
