/*
 * Border.h
 *
 *  Created on: Dec 10, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_INFLATION_BORDER_H_
#define INCLUDE_ACTOR_INFLATION_BORDER_H_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Line3.hh>
#include <visualization_msgs/Marker.h>
#include <ignition/math/Box.hh>

namespace actor {
namespace inflation {

/// \brief Constants describing the shape of a border
typedef enum {
	BORDER_NONE = 0u,//!< BORDER_NONE
	BORDER_RECTANGLE,//!< BORDER_RECTANGLE
	BORDER_CIRCLE,   //!< BORDER_CIRCLE
	BORDER_ELLIPSE   //!< BORDER_ELLIPSE
} BorderType;

/// \brief A base class for different border instances (rectangle, circle, ellipse).
/// Provides ability to call derived classes functions without a need to store
/// 3 instances of rectangular, circular and elliptical borders in memory at once.
/// \details All class methods are empty (blank) and virtual so derived classes
/// can override them while one is still able to utilize base class in the code
/// (due to lack of pure virtual methods).
class Border {

public:

	/// \brief Constructor
	Border();

	/// \brief Copy constructor
	Border(const Border &obj);

	/// \brief Returns the type of the Border, \ref BorderType
	BorderType getType() const;

	/// \brief Method which updates a class'es
	/// internal Box instance's pose
	virtual void updatePose(const ignition::math::Pose3d &new_pose);

	/// \brief Method which checks whether a box
	/// contains a given point; checks if the point
	/// lays within box'es bounds
	virtual bool doesContain(const ignition::math::Vector3d &pt) const;

	/// \section 2 versions of `doesIntersect` method are provided as a workaround
	/// for being unable to create a virtual template method. It can also be done
	/// via moving the template to a class level (\ref https://stackoverflow.com/a/27506503).
	/// \note After all, each of derived classes has `doesIntersect` method implemented.
	/// \brief Method which checks whether a given line does intersect the border
	/// \return A tuple consisting of bool flag which
	/// is true when line intersects a box and a point
	/// coordinates in which a line intersects a border;
	/// it is assumed that a line does not have
	/// 2 intersection points with the border
	virtual std::tuple<bool, ignition::math::Vector3d> doesIntersect(const ignition::math::Line3d &line) const;
	/// \brief Method which finds an intersection point
	/// based on the ellipse's center and a given point.
	/// A line passing through a pt_dest and the ellipse's
	/// center is created and one of 2 solutions is chosen
	/// to return.
	/// \param[in] pt_dest is a destination point
	/// \return A tuple consisting of bool flag,
	/// and an intersection point coordinates;
	/// one solution out of 2 is chosen
	/// \note Related to circular and elliptical borders,
	/// not usable with a rectangular border
	virtual std::tuple<bool, ignition::math::Vector3d> doesIntersect(const ignition::math::Vector3d &pt_dest) const;

	/// \section Getters
	/// \brief Returns a Box'es center coordinates
	virtual ignition::math::Vector3d getCenter() const;

	/// \brief Returns a visualization_msgs::Marker
	/// instance which is created by Box conversion
	virtual visualization_msgs::Marker getMarkerConversion() const;

	/// \brief Returns a valid ignition::math::Box instance if the Border is a rectangle
	/// (i.e. represents a 3D Box --> /isBox == true/) or a default Box instance otherwise.
	/// \note This method has been created as a workaround for the downcasting problem
	/// (dynamic_cast produced errors)
	virtual ignition::math::Box getBox() const;

	/// \brief Destructor
	virtual ~Border();

protected:

	/// \brief Represents the type of the Border instance
	BorderType type_;

};

} /* namespace inflation */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_INFLATION_BORDER_H_ */
