/*
 * Inflation.h
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_CORE_INFLATION_H_
#define INCLUDE_SFM_CORE_INFLATION_H_


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

typedef enum {
	INTERSECTION_ACTORS = 0,
	INTERSECTION_ACTOR_OBJECT
} IntersectionType;

class Inflation {

public:

	/// helper class which provides methods to calculate distances between
	/// objects taking their bounding figures into consideration
	Inflation();

	// only other objects bounding box considered
	ignition::math::Vector3d calculateModelsClosestPoints(const ignition::math::Pose3d &actor_pose,
														  const ignition::math::Pose3d &object_pose,
														  const actor::inflation::Box &bb) const;

	// actor's bounding box and object's bounding box
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> calculateModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,  const actor::inflation::Box &actor_box,
			const ignition::math::Pose3d &object_pose, const actor::inflation::Box &object_box,
			const std::string &object_name /* debug only */) const;

	/* actor's bounding Circle/Ellipse and object's Box - DEPRECATED due to error
	 * gzserver: symbol lookup error: /home/rayvburn/ros_workspace/ws_people_sim/build/actor_plugin_social/
	 * libsocial_force_model.so: undefined symbol: _ZNK3sfm4core9Inflation28calculateModelsClosestPointsIN5
	 * actor9inflation7EllipseEEESt5tupleIJN8ignition4math5Pose3IdEENS8_7Vector3IdEEEERKSA_RKT_SF_RKNS4_3Bo
	 * xERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE
	template <class T>
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> calculateModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose, const T &actor_bound,
			const ignition::math::Pose3d &object_pose,const actor::inflation::Box &object_box,
			const std::string &object_name // debug only
	) const;
	*/

	// actor's bounding Circle and object's Box
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> calculateModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose, const actor::inflation::Circle &actor_circle,
			const ignition::math::Pose3d &object_pose,const actor::inflation::Box &object_box,
			const std::string &object_name /* debug only */) const;

	/// actor's bounding Ellipse and object's Box
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> calculateModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose, const actor::inflation::Ellipse &actor_ellipse,
			const ignition::math::Pose3d &object_pose,const actor::inflation::Box &object_box,
			const std::string &object_name /* debug only */) const;

	// actors with bounding Circle
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> calculateModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,  const actor::inflation::Circle &actor_circle,
			const ignition::math::Pose3d &object_pose, const actor::inflation::Circle &object_circle,
			const std::string &_object_name /* debug only */) const;

	// actors with bounding Ellipse
	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> calculateModelsClosestPoints(
			const ignition::math::Pose3d &actor_pose,	const actor::inflation::Ellipse &actor_ellipse,
			const ignition::math::Pose3d &object_pose,	const actor::inflation::Ellipse &object_ellipse,
			const std::string &object_name /* debug only */ ) const;

	virtual ~Inflation();

private:

	// only other objects bounding box considered - actor as a point
	std::vector<ignition::math::Vector3d> createVerticesVector(const actor::inflation::Box &bb) const;
	std::vector<double> calculateLengthToVertices(const ignition::math::Vector3d &actor_pos, const std::vector<ignition::math::Vector3d> &vertices_pts) const;

	// intersected models - 2 actors or actor and box
	std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> calculateIntersectedModelsClosestPoints(
			const ignition::math::Vector3d &actor_pos, const ignition::math::Vector3d &pt_intersect,
			const IntersectionType &type) const;

};

} /* namespace core */
} /* namespace sfm */

#endif /* INCLUDE_SFM_CORE_INFLATION_H_ */
