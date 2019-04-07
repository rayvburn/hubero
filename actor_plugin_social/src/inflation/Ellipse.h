/*
 * Ellipse.h
 *
 *  Created on: Mar 31, 2019
 *      Author: rayvburn
 */

#ifndef SRC_INFLATION_ELLIPSE_H_
#define SRC_INFLATION_ELLIPSE_H_


#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <visualization_msgs/Marker.h>
#include <tuple>


namespace actor {
namespace inflation {

class Ellipse {
public:

public:

	Ellipse();
	Ellipse(const double &a_major, const double &b_minor, const double &yaw, const ignition::math::Vector3d &center_point,
					const ignition::math::Vector3d &offset_vector);
	void init(const double &a_major, const double &b_minor,  const double &yaw, const ignition::math::Vector3d &center_point,
			  const ignition::math::Vector3d &offset_vector);

	void setSemiMajorAxis(const double &a_major);
	void setSemiMinorAxis(const double &b_minor);
	void setYaw(const double &yaw_ellipse);
	void setPosition(const ignition::math::Vector3d &center_point);
	void setCenterOffset(const ignition::math::Vector3d &offset_vector);
	void updatePose(const ignition::math::Pose3d &pose);
	std::tuple<bool, ignition::math::Vector3d> getIntersection(const ignition::math::Vector3d &pt_dest) const;
	bool doesContain(const ignition::math::Vector3d &pt) const;
	ignition::math::Vector3d getCenter() const;
	ignition::math::Vector3d getCenterOffset() const;
	ignition::math::Vector3d getCenterShifted() const;
	visualization_msgs::Marker getMarkerConversion() const;

	virtual ~Ellipse();

private:

	std::tuple<ignition::math::Vector3d, double> getIntersectionExtended(const ignition::math::Vector3d &pt_dest) const;
	std::tuple<unsigned int, ignition::math::Vector3d, ignition::math::Vector3d> getIntersectionWithLine(const double &to_dest_angle) const;
	void updateShiftedCenter();
	void updateCenter();

	double a_major_;
	double b_minor_;
	double yaw_ellipse_;
	double yaw_offset_;

	ignition::math::Vector3d center_;
	ignition::math::Vector3d offset_;
	ignition::math::Vector3d center_shifted_; // shifted center of the ellipse (center + offset with current yaw taken into consideration)

};

} /* namespace inflation */
} /* namespace actor */

#endif /* SRC_INFLATION_ELLIPSE_H_ */
