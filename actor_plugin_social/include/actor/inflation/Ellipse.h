/*
 * Ellipse.h
 *
 *  Created on: Mar 31, 2019
 *      Author: rayvburn
 */

#ifndef SRC_INFLATION_ELLIPSE_H_
#define SRC_INFLATION_ELLIPSE_H_

#include <actor/inflation/Border.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <visualization_msgs/Marker.h>
#include <tuple>


namespace actor {
namespace inflation {

/// \brief A class which creates a elliptical
/// `inflation` figure around the actor;
/// used by SFM
class Ellipse : public Border {

public:

	/// \brief Default constructor
	Ellipse();

	/// \brief Constructor with explicitly passed parameters
	Ellipse(const double &a_major, const double &b_minor, const double &yaw, const ignition::math::Vector3d &center_point,
					const ignition::math::Vector3d &offset_vector);

	/// \brief Method which configures the ellipse's parameters
	void init(const double &a_major, const double &b_minor,  const double &yaw, const ignition::math::Vector3d &center_point,
			  const ignition::math::Vector3d &offset_vector);

	/// \brief Sets a semi-major axis length
	void setSemiMajorAxis(const double &a_major);

	/// \brief Sets a semi-minor axis length
	void setSemiMinorAxis(const double &b_minor);

	/// \brief Sets a rotation of the ellipse
	void setYaw(const double &yaw_ellipse);

//	/// \brief Sets a shifted center of the ellipse;
//	/// actual center point (semi-axes intersection)
//	/// is calculated independently
//	/// FIXME: only the translation component is considered
//	void updatePose(const ignition::math::Pose3d &new_pose);

	/// \brief Sets an offset of a center;
	/// in terms of vectors a shifted center
	/// is expressed as a sum of a center
	/// and the offset
	void setCenterOffset(const ignition::math::Vector3d &offset_vector);

	/// \brief Updates the pose of the ellipse
	/// (position and orientation)
	virtual void updatePose(const ignition::math::Pose3d &pose) override;

	/// \brief Method which finds an intersection point
	/// based on the ellipse's center and a given point.
	/// A line passing through a pt_dest and the ellipse's
	/// center is created and one of 2 solutions is chosen
	/// to return.
	/// \param[in] pt_dest is a destination point
	/// \return A tuple consisting of bool flag,
	/// and an intersection point coordinates;
	/// one solution out of 2 is chosen
	virtual std::tuple<bool, ignition::math::Vector3d> doesIntersect(const ignition::math::Vector3d &pt_dest) const override;

	/// \brief Method which checks whether the ellipse
	/// contains a given point; checks if the point
	/// lays within ellipse's bounds
	virtual bool doesContain(const ignition::math::Vector3d &pt) const override;

	/// \brief Returns an actual center of the ellipse -
	/// not the shifted one
	virtual ignition::math::Vector3d getCenter() const override;

	/// \brief Returns an center's offset of the ellipse
	ignition::math::Vector3d getCenterOffset() const;

	/// \brief Returns a shifted center of the ellipse
	ignition::math::Vector3d getCenterShifted() const;

	/// \brief Returns a visualization_msgs::Marker
	/// instance which is created by ellipse conversion;
	/// the height of a model is equal to 1.8 m
	virtual visualization_msgs::Marker getMarkerConversion() const override;

	/// \brief Default destructor
	virtual ~Ellipse();

private:

	/* DEPRECATED */
	std::tuple<ignition::math::Vector3d, double> getIntersectionExtended(const ignition::math::Vector3d &pt_dest) const;

	/// \brief Helper method which finds 0, 1 or 2 intersection points
	/// of a line with the ellipse
	/// \return A tuple consisting of boolean flag
	/// which is true when at least 1 intersection
	/// point is found, and 2 position vectors
	/// which both are non-zero when 2 solutions
	/// are found
	std::tuple<unsigned int, ignition::math::Vector3d, ignition::math::Vector3d> getIntersectionWithLine(const double &to_dest_angle) const;

	/* DEPRECATED */
	void updateShiftedCenter();

	/// \brief Helper method which calculates an actual
	/// center of the ellipse based on its shifted center
	/// and orientation (yaw angle)
	void updateCenter();

	/// \brief Stores a semi-major axis length
	double a_major_;

	/// \brief Stores a semi-minor axis length
	double b_minor_;

	/// \brief Stores a yaw angle (orientation)
	double yaw_ellipse_;

	/* DEPRECATED */
	double yaw_offset_;

	/// \brief Center of the ellipse; center
	/// is calculated based on center_shifted
	/// with ellipse's orientation and ellipse's
	/// offset taken into consideration
	ignition::math::Vector3d center_;

	/// \brief Center offset vector
	ignition::math::Vector3d offset_;

	/// \brief A shifted center of the ellipse;
	/// equal to the current position of an actor
	ignition::math::Vector3d center_shifted_;

};

} /* namespace inflation */
} /* namespace actor */

#endif /* SRC_INFLATION_ELLIPSE_H_ */
