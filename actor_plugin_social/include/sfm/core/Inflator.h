/*
 * Inflation.h
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_CORE_INFLATOR_H_
#define INCLUDE_SFM_CORE_INFLATOR_H_

#include <actor/inflation/Border.h>
#include <actor/inflation/Box.h>
//#include <actor/inflation/Circle.h>
//#include <actor/inflation/Ellipse.h>
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
	static ignition::math::Vector3d findClosestPointsModelBox(const ignition::math::Pose3d &actor_pose,
														  const ignition::math::Pose3d &object_pose,
														  const actor::inflation::Border &bb);

	/// \brief Function used when an actor's Bounding Box
	/// and an object's Bounding Box are considered
	static std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findClosestPointsBoxes(
			const ignition::math::Pose3d &actor_pose,  const actor::inflation::Border &actor_box,
			const ignition::math::Pose3d &object_pose, const actor::inflation::Border &object_box,
			const std::string &object_name /* debug only */);

	/* Scroll down for an additional template definition which is used
	 * for Bounding Circle / Bounding Ellipse and Bounding Box computations */

	/// \brief Function used when 2 actors
	/// with Bounding Circles are considered
	static std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findClosestPointsCircles(
			const ignition::math::Pose3d &actor_pose,  const actor::inflation::Border &actor_circle,
			const ignition::math::Pose3d &object_pose, const actor::inflation::Border &object_circle,
			const std::string &_object_name /* debug only */);

	/// \brief Function used when 2 actors
	/// with Bounding Ellipses are considered
	static std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findClosestPointsEllipses(
			const ignition::math::Pose3d &actor_pose,	const actor::inflation::Border &actor_ellipse,
			const ignition::math::Pose3d &object_pose,	const actor::inflation::Border &object_ellipse,
			const std::string &object_name /* debug only */ );

	/// \brief Default destructor
	virtual ~Inflator();

private:

	/// \brief Helper function used when only other objects' bounding
	/// box is considered and an actor is treated as a point;
	/// creates a vector of vertices of a Bounding Box;
	/// \return A vector of 4 vertices of a rectangle is created -
	/// Bounding Box is considered to be planar here
	static std::vector<ignition::math::Vector3d> createVerticesVector(const ignition::math::Box &bb);

	/// \brief Helper function used when only other objects' bounding
	/// box is considered and an actor is treated as a point;
	/// calculates distance from actor's position to each vertex
	/// of a bounding box
	static std::vector<double> calculateLengthToVertices(const ignition::math::Vector3d &actor_pos, const std::vector<ignition::math::Vector3d> &vertices_pts);

	/// \brief Helper function used when models intersection occurs;
	/// incorporates 2 cases of intersection: 2 actors or actor
	/// an an object
	static std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> findIntersectedModelsClosestPoints(
			const ignition::math::Vector3d &actor_pos, const ignition::math::Vector3d &pt_intersect,
			const IntersectionType &type);

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
	static std::tuple<bool, ignition::math::Vector3d> isWithinRangeOfBB(
			const ignition::math::Vector3d &actor_center,
			const actor::inflation::Box &object_bb);

	/// Used for function output representation.
	/// TODO: Copy diagrams to doxygen documentation
	typedef enum {
		BOX_INTERSECTION_TYPE_PARTIAL_LOWER = 0,//!< BOX_INTERSECTION_TYPE_PARTIAL_LOWER: diagram 3
		BOX_INTERSECTION_TYPE_PARTIAL_UPPER,    //!< BOX_INTERSECTION_TYPE_PARTIAL_UPPER: diagram 4
		BOX_INTERSECTION_TYPE_INTERNAL,         //!< BOX_INTERSECTION_TYPE_INTERNAL: diagram 1
		BOX_INTERSECTION_TYPE_EXTERNAL,         //!< BOX_INTERSECTION_TYPE_EXTERNAL: diagram 5
		BOX_INTERSECTION_TYPE_NONE_LOWER,       //!< BOX_INTERSECTION_TYPE_NONE_LOWER: diagram 2
		BOX_INTERSECTION_TYPE_NONE_UPPER,       //!< BOX_INTERSECTION_TYPE_NONE_UPPER: diagram 6
		BOX_INTERSECTION_TYPE_UNKNOWN			//!< BOX_INTERSECTION_TYPE_UNKNOWN
	} BoxIntersectionType;

	/// \brief Used in the reasoning block. Determine whether 2 objects share
	/// some space (BOXES_SURFACE_INTERSECTION_PARTIAL)
	/// or share only single coordinate (BOXES_SURFACE_INTERSECTION_SPECIAL)
	/// or no intersection occurs
	typedef enum {
		BOXES_SURFACE_INTERSECTION_NONE = 0,//!< BOXES_SURFACE_INTERSECTION_NONE
		BOXES_SURFACE_INTERSECTION_SPECIAL, //!< BOXES_SURFACE_INTERSECTION_SPECIAL
		BOXES_SURFACE_INTERSECTION_PARTIAL  //!< BOXES_SURFACE_INTERSECTION_PARTIAL
	} BoxesSurfaceIntersection;

	/// \brief Helper method used for finding the location of the obstacle relative to the actor.
	/// \param actor_coord_min
	/// \param actor_coord_max
	/// \param object_coord_min
	/// \param object_coord_max
	/// \return See \ref BoxIntersectionType enum
	static BoxIntersectionType findIntersectionType(const double &actor_coord_min, const double &actor_coord_max,
			const double &object_coord_min, const double &object_coord_max);

	/// \brief Finds the type of intersection occurring between 2 boxes (the one associated
	/// with an actor and the one associated with an obstacle).
	/// \param actor_bb
	/// \param object_bb
	/// \return A tuple:
	/// Enum variable defining whether the shape surfaces intersect;
	/// Enum variable defining the special intersection type associated with the X-axis (does not matter if first enum is different than BOXES_SURFACE_INTERSECTION_SPECIAL);
	/// Enum variable defining the special intersection type associated with the Y-axis (does not matter if first enum is different than BOXES_SURFACE_INTERSECTION_SPECIAL);
	static std::tuple<BoxesSurfaceIntersection, Inflator::BoxIntersectionType, Inflator::BoxIntersectionType>
	findIntersectionTypeOfBB(const actor::inflation::Box &actor_bb, const actor::inflation::Box &object_bb);

	/// \brief
	/// \param x_intersection: a value of BoxIntersectionType
	/// \param y_intersection: a value of BoxIntersectionType
	/// \param actor_bb
	/// \param object_bb
	/// \return A tuple:
	/// Actor point that the shortest vector is generated from
	/// Obstacle point that the shortest vector is generated from
	static std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>
	findVectorSpecialIntersectionBB(const Inflator::BoxIntersectionType &x_intersection,
									const Inflator::BoxIntersectionType &y_intersection,
									const actor::inflation::Box &actor_bb,
									const actor::inflation::Box &object_bb);

	/// \brief Tries to find an intersection point starting in the `object_pos`,
	/// ending in the `box`'es center or `box_pos` if given (default value provided).
	/// The intersection point is calculated via BB'es `Intersect` method
	/// taking an ignition::math::Line3 instance.
	/// \param[in] object_pos: a point which must be located outside of the BoundingBox,
	/// otherwise computations will fail and no intersection will be found.
	/// \note `box_pos` must be within `box` bounds, otherwise the center point
	/// will be considered in calculations.
	static ignition::math::Vector3d calculateBoxIntersection(const ignition::math::Vector3d &object_pos,
			const actor::inflation::Border &box,
			const ignition::math::Vector3d &box_pos = ignition::math::Vector3d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()));

	/// \brief Helper method which finds the closest Bounding
	/// Box vertex relative to the given position (`actor_pos`)
	/// \return Vector of the closest vertex coordinates
	static ignition::math::Vector3d findClosestBoundingBoxVertex(const ignition::math::Vector3d &actor_pos,
			const actor::inflation::Border &object_box);

	/// \brief Finds the closest vertices of the 2 boxes. Does not evaluate whether the boxes coordinates intersect
	/// \param actor_box
	/// \param object_box
	/// \return A tuple: start and end points of the vector connecting the closest vertices
	static std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> findClosestVerticesIntersectedBoxes(const actor::inflation::Box &actor_box, const actor::inflation::Box &object_box);

	/// \brief Takes a common (shared) coordinate value and calculates the shortest vector
	/// connecting 2 boxes (rectangular borders) that share the space in terms of a single
	/// axis. Requires an `axis` to be specified.
	/// \param axis
	/// \param coord_common
	/// \param actor_box
	/// \param object_box
	/// \return A tuple containing a positions of the actor and the obstacle (in that order)
	static std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> findShortestVectorIntersectedBoxes(const char &axis, const double &coord_common, const actor::inflation::Box &actor_box, const actor::inflation::Box &object_box);

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
	static std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findClosestPointsActorBox(
			const ignition::math::Pose3d &actor_pose, const T &actor_bound,
			const ignition::math::Pose3d &object_pose,const actor::inflation::Border &object_box,
			const std::string &object_name /* debug only */	) {

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
		std::tie(within_bb, object_anchor) = isWithinRangeOfBB(actor_pose.Pos(), object_box.getBox()); // note: without the `isBox` evaluation

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

		if ( actor_bound.getType() == actor::inflation::BORDER_RECTANGLE ) {
			// find a point of an intersection of the `line` and actor's box (`actor_bound`)
			ignition::math::Line3d line(object_pos_shifted, actor_pose_shifted.Pos());
			std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_bound.doesIntersect(line);
		} else {
			// intersection of the actor's circle
			std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_bound.doesIntersect(object_pos_shifted);
		}
		return (std::make_tuple(actor_pose_shifted, object_pos_shifted));

	} /* findModelsClosestPoints() */


}; /* class Inflator */

} /* namespace sfm */

#endif /* INCLUDE_SFM_CORE_INFLATOR_H_ */
