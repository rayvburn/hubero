/*
 * Inflation.h
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_CORE_INFLATOR_H_
#define INCLUDE_SFM_CORE_INFLATOR_H_


#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Line3.hh>
#include <ignition/math/Angle.hh>

#include "inflation/Box.h"
#include "inflation/Circle.h"
#include "inflation/Ellipse.h"

#include <tuple>
#include <string>
#include <vector>


namespace sfm {
namespace core {

// ---------------------------------

typedef enum {
	INTERSECTION_ACTORS = 0,
	INTERSECTION_ACTOR_OBJECT
} IntersectionType;

// ---------------------------------

class Inflator {

public:

	/// helper class which provides methods to calculate distances between
	/// objects taking their bounding figures into consideration
	Inflator();

	// only other objects bounding box considered
	ignition::math::Vector3d findModelsClosestPoints(const ignition::math::Pose3d &actor_pose,
														  const ignition::math::Pose3d &object_pose,
														  const actor::inflation::Box &bb) const;

	// actor's bounding box and object's bounding box
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,  const actor::inflation::Box &actor_box,
			const ignition::math::Pose3d &object_pose, const actor::inflation::Box &object_box,
			const std::string &object_name /* debug only */) const;

//	// actor's bounding Circle and object's Box
//	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
//			const ignition::math::Pose3d &actor_pose, const actor::inflation::Circle &actor_circle,
//			const ignition::math::Pose3d &object_pose,const actor::inflation::Box &object_box,
//			const std::string &object_name /* debug only */) const;
//
//	/// actor's bounding Ellipse and object's Box
//	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
//			const ignition::math::Pose3d &actor_pose, const actor::inflation::Ellipse &actor_ellipse,
//			const ignition::math::Pose3d &object_pose,const actor::inflation::Box &object_box,
//			const std::string &object_name /* debug only */) const;


	// actors with bounding Circle
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,  const actor::inflation::Circle &actor_circle,
			const ignition::math::Pose3d &object_pose, const actor::inflation::Circle &object_circle,
			const std::string &_object_name /* debug only */) const;

	// actors with bounding Ellipse
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,	const actor::inflation::Ellipse &actor_ellipse,
			const ignition::math::Pose3d &object_pose,	const actor::inflation::Ellipse &object_ellipse,
			const std::string &object_name /* debug only */ ) const;

	virtual ~Inflator();

private:

	// only other objects bounding box considered - actor as a point
	std::vector<ignition::math::Vector3d> createVerticesVector(const actor::inflation::Box &bb) const;
	std::vector<double> calculateLengthToVertices(const ignition::math::Vector3d &actor_pos, const std::vector<ignition::math::Vector3d> &vertices_pts) const;

	// intersected models - 2 actors or actor and box
	std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> findIntersectedModelsClosestPoints(
			const ignition::math::Vector3d &actor_pos, const ignition::math::Vector3d &pt_intersect,
			const IntersectionType &type) const;

public:

	/* actor's bounding Circle/Ellipse and object's Box - DEPRECATED due to error
	 * gzserver: symbol lookup error: /home/rayvburn/ros_workspace/ws_people_sim/build/actor_plugin_social/
	 * libsocial_force_model.so: undefined symbol: _ZNK3sfm4core9Inflation28calculateModelsClosestPointsIN5
	 * actor9inflation7EllipseEEESt5tupleIJN8ignition4math5Pose3IdEENS8_7Vector3IdEEEERKSA_RKT_SF_RKNS4_3Bo
	 * xERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE */

	template <class T>
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> findModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose, const T &actor_bound,
			const ignition::math::Pose3d &object_pose,const actor::inflation::Box &object_box,
			const std::string &object_name /* debug only */	) const {

		/* BoundingEllipse and BoundingBox -> actor and static object */

		/* this function finds points that are located within the bounding
		 * box/circle range that are further treated as a real position
		 * of objects in the world - this provides some kind of an inflation
		 * around the objects */

		ignition::math::Pose3d actor_pose_shifted = actor_pose;
		ignition::math::Line3d line;
		ignition::math::Vector3d point_intersect;

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmGetPrintData() ) {
		if ( debug_current_actor_name == "actor1" ) {
			std::cout << "\n---------------------------------------------------------------------------------\n";
			std::cout << "in GetActorModelBBsClosestPoints() - ACTOR & OBJECT - checking intersection\n";
			std::cout << "\t" << debug_current_actor_name << "'s pos: " << actor_pose.Pos() << "\t" << debug_current_object_name << "'s pos: " << object_pose.Pos() << std::endl;
		}
		}
		#endif

		// object's bounding box point that is closest to actor's bounding box
		/* create the line from the actor's center to the object's center and check
		 * the intersection point of that line with the object's bounding box */
		line.Set(actor_pose.Pos().X(), actor_pose.Pos().Y(), object_pose.Pos().X(), object_pose.Pos().Y(), object_box.getCenter().Z() );
		bool intersects = false;
		std::tie(intersects, point_intersect) = object_box.doesIntersect(line);

		/* bb's Intersect returns intersection point whose X and Y are equal
		 * to actor's X and Y thus 0 distance between points and no solutions
		 * are generated in quadratic equation (happens when actor steps into
		 * obstacle; as a hack - set intersection point as a object's center */
		if ( (point_intersect.X() == actor_pose.Pos().X()) && (point_intersect.Y() == actor_pose.Pos().Y()) ) {
			point_intersect = object_pose.Pos();
		}

		// when centers are aligned, move a little in arbitrary direction
		// just to avoid zeroing force
		if ( (point_intersect.X() == actor_pose.Pos().X()) && (point_intersect.Y() == actor_pose.Pos().Y()) ) {
			point_intersect.X( point_intersect.X() + 0.005 ); point_intersect.Y( point_intersect.Y() - 0.005 );
		}


		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmGetPrintData() ) {
		if ( debug_current_actor_name == "actor1" ) {
			std::cout << "\n\tObject's BBox intersection result: " << intersects << "\tpt: "<< point_intersect;
			std::cout << "\n\tline len: " << line.Length();
		}
		}
		#endif

		/* now there is an ability to check whether actor has stepped into some obstacle
		 * (its bounding box - to be specific) */

		if ( actor_bound.doesContain(point_intersect) ) {

			#if defined(DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX_BB) || defined(DEBUG_BOUNDING_ELLIPSE_INTERSECTION)
			if ( SfmGetPrintData() ) {
			std::cout << "\n\n\n\n\n1\tACTOR STEPPED INTO OBSTACLE\n\n\n\n\n\t" << debug_current_actor_name << "\t" << debug_current_object_name << "\n" << std::endl;
			}
			#endif
			std::tie(actor_pose_shifted.Pos(), point_intersect) = findIntersectedModelsClosestPoints(actor_pose.Pos(), point_intersect, INTERSECTION_ACTOR_OBJECT);

			#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
			if ( SfmGetPrintData() ) {
			if ( debug_current_actor_name == "actor1" ) {
				std::cout << "\n---------------------------------------------------------------------------------\n\n\n";
			}
			}
			#endif

			return ( std::make_tuple(actor_pose_shifted, point_intersect) );

		}

		#ifdef DEBUG_CLOSEST_POINTS
		if ( _object_name == "table1" ) {
			std::cout << "GetClosestPoints()\t" << _object_name << "'s BB point of intersection: " << point_intersect;
		}
		#endif

		// intersection of the actor's circle
		std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_bound.getIntersection(point_intersect);

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmGetPrintData() ) {
		if ( debug_current_actor_name == "actor1" ) {
			std::cout << "\nactor's BE intersection result - new actor's pose: " << actor_pose_shifted.Pos();
		}
		}
		#endif

		#ifdef DEBUG_CLOSEST_POINTS
		if ( _object_name == "table1" ) {
			std::cout << "\t\tACTOR's BC point of intersection: " << actor_pose_shifted.Pos() << std::endl;
		}
		#endif


		#ifdef DEBUG_BOUNDING_CIRCLE
		std::cout << "\n\nBOUND - actor & object | actor pos: " << _actor_pose.Pos() << "\tintersection: " << actor_pose_shifted.Pos() << std::endl;
		std::cout << "BOUND - actor & object |" << _object_name << "'s pos: " << _object_pose.Pos() << "\tintersection: " << point_intersect << std::endl;
		#endif

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmGetPrintData() ) {
		if ( debug_current_actor_name == "actor1" ) {
			std::cout << "\n---------------------------------------------------------------------------------\n\n\n";
		}
		}
		#endif

		return ( std::make_tuple(actor_pose_shifted, point_intersect) );

	}


};

} /* namespace core */
} /* namespace sfm */

#endif /* INCLUDE_SFM_CORE_INFLATOR_H_ */
