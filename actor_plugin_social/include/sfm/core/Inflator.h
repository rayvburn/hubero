/*
 * Inflation.h
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_CORE_INFLATOR_H_
#define INCLUDE_SFM_CORE_INFLATOR_H_


#include <actor/inflation/Box.h>
#include <actor/inflation/Circle.h>
#include <actor/inflation/Ellipse.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Angle.hh>
#include <sfm/core/SFMDebug.h>
#include <tuple>
#include <string>
#include <vector>
#include <limits> 	// std::numeric_limits
#include <iterator> // std::distance

// template definition placed in header - load directives
// which parts of code to compile

namespace sfm {

// ---------------------------------

typedef enum {
	INTERSECTION_ACTORS = 0,
	INTERSECTION_ACTOR_OBJECT
} IntersectionType;

// ---------------------------------

class Inflator {

public:

	/// \brief Class which provides methods to calculate distances
	/// between objects taking their bounding models into consideration
	Inflator();

	/// \brief Function used when only other objects' bounding
	/// box is considered and an actor is treated as a point
	ignition::math::Vector3d findModelsClosestPoints(const ignition::math::Pose3d &actor_pose,
														  const ignition::math::Pose3d &object_pose,
														  const actor::inflation::Box &bb) const;

	/// \brief Function used when an actor's Bounding Box
	/// and an object's Bounding Box are considered
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,  const actor::inflation::Box &actor_box,
			const ignition::math::Pose3d &object_pose, const actor::inflation::Box &object_box,
			const std::string &object_name /* debug only */) const;

	/* Scroll down for an additional template definition which is used
	 * for Bounding Circle / Bounding Ellipse and Bounding Box computations */

	/// \brief Function used when 2 actors
	/// with Bounding Circles are considered
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,  const actor::inflation::Circle &actor_circle,
			const ignition::math::Pose3d &object_pose, const actor::inflation::Circle &object_circle,
			const std::string &_object_name /* debug only */) const;

	/// \brief Function used when 2 actors
	/// with Bounding Ellipses are considered
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,	const actor::inflation::Ellipse &actor_ellipse,
			const ignition::math::Pose3d &object_pose,	const actor::inflation::Ellipse &object_ellipse,
			const std::string &object_name /* debug only */ ) const;

	/// \brief Default destructor
	virtual ~Inflator();

private:

	/// \brief Helper function used when only other objects' bounding
	/// box is considered and an actor is treated as a point;
	/// creates a vector of vertices of a Bounding Box;
	/// \return A vector of 4 vertices of a rectangle is created -
	/// Bounding Box is considered to be planar here
	std::vector<ignition::math::Vector3d> createVerticesVector(const actor::inflation::Box &bb) const;

	/// \brief Helper function used when only other objects' bounding
	/// box is considered and an actor is treated as a point;
	/// calculates distance from actor's position to each vertex
	/// of a bounding box
	std::vector<double> calculateLengthToVertices(const ignition::math::Vector3d &actor_pos, const std::vector<ignition::math::Vector3d> &vertices_pts) const;

	/// \brief Helper function used when models intersection occurs;
	/// incorporates 2 cases of intersection: 2 actors or actor
	/// an an object
	std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> findIntersectedModelsClosestPoints(
			const ignition::math::Vector3d &actor_pos, const ignition::math::Vector3d &pt_intersect,
			const IntersectionType &type) const;

	/// \brief Helper function used to evaluate if actor's center is located
	/// within the object's BB range (i.e. actor center coordinates
	/// are located within the object coordinates range).
	/// This does not mean that actor's center is located in the space
	/// bounded by the box edges!
	/// \return A tuple consisting of bool (True if actor center is within
	/// the object BB range) and Vector3 (object BB'es point
	/// located somewhere on the edge (if bool is True)
	/// or in the center (if bool is False))
	/// \note Extremely useful for objects like long walls.
	std::tuple<bool, ignition::math::Vector3d> isWithinRangeOfBB(
			const ignition::math::Vector3d &actor_center,
			const actor::inflation::Box &object_bb) const;

	/// \brief Tries to find an intersection point starting in the `object_pos`,
	/// ending in the `box`'es center or `box_pos` if given (default value provided).
	/// The intersection point is calculated via BB'es `Intersect` method
	/// taking an ignition::math::Line3 instance.
	/// \param[in] object_pos: a point which must be located outside of the BoundingBox,
	/// otherwise computations will fail and no intersection will be found.
	/// \note `box_pos` must be within `box` bounds, otherwise the center point
	/// will be considered in calculations.
	ignition::math::Vector3d calculateBoxIntersection(const ignition::math::Vector3d &object_pos,
			const actor::inflation::Box &box,
			const ignition::math::Vector3d &box_pos = ignition::math::Vector3d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN())) const;

	/// \brief Helper method which finds the closest Bounding
	/// Box vertex relative to the given position (`actor_pos`)
	/// \return Vector of the closest vertex coordinates
	ignition::math::Vector3d findClosestBoundingBoxVertex(const ignition::math::Vector3d &actor_pos, const actor::inflation::Box &object_box) const;

public:

	/* Template definition in cpp threw an error:
	 * gzserver: symbol lookup error: /home/rayvburn/ros_workspace/ws_people_sim/build/actor_plugin_social/
	 * libsocial_force_model.so: undefined symbol: _ZNK3sfm4core9Inflation28calculateModelsClosestPointsIN5
	 * actor9inflation7EllipseEEESt5tupleIJN8ignition4math5Pose3IdEENS8_7Vector3IdEEEERKSA_RKT_SF_RKNS4_3Bo
	 * xERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE */

	/// \brief Function used for calculations when an Actor's
	/// Bounding Circle/Bounding Ellipse and an object's Box
	/// are considered
	template <class T>
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose, const T &actor_bound,
			const ignition::math::Pose3d &object_pose,const actor::inflation::Box &object_box,
			const std::string &object_name /* debug only */	) const {

		/* BoundingCircle and BoundingBox  -> actor and static object */
		/* BoundingEllipse and BoundingBox -> actor and static object */

		/* This function finds points that are located on the edges of the bounding
		 * box/circle. Later on they are later on treated as real positions
		 * of objects in the world - this provides some kind of an inflation
		 * around objects. */

		// allocate shifted poses/positions, their position components will likely be updated
		ignition::math::Pose3d actor_pose_shifted = actor_pose;
		ignition::math::Vector3d object_pos_shifted = object_pose.Pos();

		// flag indicating if actor's position is within BB edge range
		// (x_actor >= x_edge_min && x_actor <= x_edge_max)
		bool within_bb = false;

		// represents an object's BOX reference point, it can be equal
		// to the `object_pose`'s position or shifted along X/Y axis;
		// for details see @ref isWithinRangeOfBB
		ignition::math::Vector3d object_anchor;
		std::tie(within_bb, object_anchor) = isWithinRangeOfBB(actor_pose.Pos(), object_box);

		// `within_bb` is TRUE if `object_anchor` is not equal to the `object_box`'s center
		if ( within_bb ) {
			// find object shifted position (intersection)
			object_pos_shifted = calculateBoxIntersection(actor_bound.getCenter(), object_box, object_anchor);
		} else {
			// try to find the closest vertex of the Box
			object_pos_shifted = findClosestBoundingBoxVertex(actor_bound.getCenter(), object_box);
		}

		/* bb's Intersect returns intersection point whose X and Y are equal
		 * to actor's X and Y thus 0 distance between points and no solutions
		 * are generated in quadratic equation (happens when actor steps into
		 * obstacle; as a hack - set intersection point as an object's center */
		// NOTE: this is an emergency stuff, shouldn't happen at all
		if ( (object_pos_shifted.X() == actor_pose.Pos().X()) && (object_pos_shifted.Y() == actor_pose.Pos().Y()) ) {
			object_pos_shifted = object_pose.Pos();
			// when centers are aligned, move a little in arbitrary direction
			// just to avoid zeroing force
			object_pos_shifted.X( object_pos_shifted.X() + 0.005 );
			object_pos_shifted.Y( object_pos_shifted.Y() - 0.005 );
		}

		/* now there is an ability to check whether actor has stepped into some obstacle
		 * (its bounding box - to be specific) */
		if ( actor_bound.doesContain(object_pos_shifted) ) {

			std::tie(actor_pose_shifted.Pos(), object_pos_shifted) = findIntersectedModelsClosestPoints(actor_pose.Pos(), object_pos_shifted, INTERSECTION_ACTOR_OBJECT);
			return ( std::make_tuple(actor_pose_shifted, object_pos_shifted) );

		}

		// intersection of the actor's circle
		std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_bound.doesIntersect(object_pos_shifted);
		return (std::make_tuple(actor_pose_shifted, object_pos_shifted));

	} /* findModelsClosestPoints() */


}; /* class Inflator */

} /* namespace sfm */

#endif /* INCLUDE_SFM_CORE_INFLATOR_H_ */
