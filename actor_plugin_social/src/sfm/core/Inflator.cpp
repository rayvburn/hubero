/*
 * Inflator.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#include <ignition/math/Line3.hh>
#include <sfm/core/Inflator.h>
#include <algorithm> // std::min_element

namespace sfm {

// ------------------------------------------------------------------- //

Inflator::Inflator() { }

// ------------------------------------------------------------------- //

ignition::math::Vector3d Inflator::findClosestPointsModelBox(const ignition::math::Pose3d &actor_pose,
		const ignition::math::Pose3d &object_pose, const actor::inflation::Border &bb) const
{

	// TODO: check operation
	// it doesn't take ACTOR's BOUNDING BOX into consideration (only object's BB)

	/* Assuming axis-aligned bounding box - closest point search algorithm:
	 * 1st case:
	 * 		o 	check whether actor's y-coordinate-defined line intersects the bounding box - if yes, then the y-coordinate
	 * 			of the closest point is already known and x-coordinate will be located on the closest to actor
	 * 			edge of a bounding box
	 * 2nd case:
	 * 		o 	analogical to 1st one but first check is connected with x-coordinate intersection with bounding box
	 * 3rd case:
	 * 		o 	none of actor's coordinates intersect the bounding box - let's check 4 vertices (assuming on-plane
	 * 			check) and choose the closest one
	 */

	// inf has an object with no bounding box defined (for example - actor)
	if ( std::fabs(bb.getCenter().X()) > 1e+300 ) {
		// another actor met
		return ( ignition::math::Vector3d(object_pose.Pos()) );
	}

	// BB's Intersect() method returns a tuple
	bool does_intersect = false;
	double dist_intersect = 0.0;
	ignition::math::Vector3d point_intersect;

	// 0 case  -------------------------------------------------------------------
	// if actor stepped into some obstacle then force its central point to be the closest - WIP
	// otherwise actor will not see the obstacle (BEHIND flag will be raised)
	if ( bb.doesContain(actor_pose.Pos()) ) {
		return (bb.getCenter());
	}

	// 1st case -------------------------------------------------------------------
	ignition::math::Line3d line;
	// create a line of which intersection with a bounding box will be checked, syntax: x1, y1, x2, y2, z_common
//	line.Set(-1e+50, actor_pose.Pos().Y(), +1e+50, actor_pose.Pos().Y(), bb.Center().Z() );
	line.Set(actor_pose.Pos().X(), actor_pose.Pos().Y(), bb.getCenter().X(), actor_pose.Pos().Y(), bb.getCenter().Z() );
	std::tie(does_intersect, point_intersect) = bb.doesIntersect(line);

	if ( does_intersect ) {
		return (point_intersect);
	}

	// 2nd case -------------------------------------------------------------------
//	line.Set(actor_pose.Pos().X(), -1e+50, actor_pose.Pos().X(), +1e+50, bb.Center().Z() );
	line.Set(actor_pose.Pos().X(), actor_pose.Pos().Y(), actor_pose.Pos().X(), bb.getCenter().Y(), bb.getCenter().Z() );
	std::tie(does_intersect, point_intersect) = bb.doesIntersect(line);

	if ( does_intersect ) {
		return (point_intersect);
	}

	// 3rd case -------------------------------------------------------------------
	return (findClosestBoundingBoxVertex(actor_pose.Pos(), bb));

}

// ------------------------------------------------------------------- //

std::vector<ignition::math::Vector3d> Inflator::createVerticesVector(const ignition::math::Box &bb) const
{
	// 4 vertices only (planar)
	std::vector<ignition::math::Vector3d> temp_container;
	ignition::math::Vector3d temp_vector;

	// planar objects considered, height does not matter
	// as long as it is the same for both vector points
	temp_vector.Z(bb.Center().Z());

	temp_vector.X(bb.Min().X()); 	temp_vector.Y(bb.Min().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.Min().X()); 	temp_vector.Y(bb.Max().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.Max().X()); 	temp_vector.Y(bb.Min().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.Max().X()); 	temp_vector.Y(bb.Max().Y());
	temp_container.push_back(temp_vector);

	// emplace_back() makes system stuck completely
	return (temp_container);

}

// ------------------------------------------------------------------- //

std::vector<double> Inflator::calculateLengthToVertices(const ignition::math::Vector3d &actor_pos,
		const std::vector<ignition::math::Vector3d> &vertices_pts) const
{

	std::vector<double> temp_containter;
	for ( size_t i = 0; i < vertices_pts.size(); i++ ) {
		// planar
		ignition::math::Vector3d v_plane = vertices_pts.at(i) - actor_pos; 	v_plane.Z(0.0);
		temp_containter.push_back(v_plane.Length());
	}
	return (temp_containter);

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findClosestPointsBoxes(
		const ignition::math::Pose3d &actor_pose,  const actor::inflation::Border &actor_box,
		const ignition::math::Pose3d &object_pose, const actor::inflation::Border &object_box,
		const std::string &_object_name /* debug only */ ) const
{

	// allocate shifted poses/positions, their position components will likely be updated
	ignition::math::Pose3d actor_pose_shifted = actor_pose;
	ignition::math::Pose3d object_pose_shifted = object_pose;

	// ----------------------------------------------------
	// 1st case: actor's center is within object BB's range
	// (i.e. x_actor <= x_bb_max, x_actor >= x_bb_min,
	// analogically along the Y-axis)
	//
	// 2nd case: actor's center is not within object BB's range
	// (i.e. x_actor <= x_bb_max, x_actor >= x_bb_min,
	// analogically along the Y-axis)

	bool within_bb = false;
	// represents an object's BOX reference point, it can be equal
	// to the `object_pose`'s position or shifted along X/Y axis;
	// for details see @ref isWithinRangeOfBB
	ignition::math::Vector3d object_anchor;
	std::tie(within_bb, object_anchor) = isWithinRangeOfBB(actor_pose.Pos(), object_box.getBox());
	// `within_bb` is TRUE if `object_anchor` is not equal to the `object_box`'s center

	// find object shifted position (intersection)
	object_pose_shifted.Pos() = calculateBoxIntersection(actor_pose.Pos(), object_box, object_anchor);

	// find actor shifted position (intersection);
	// the line has to be 'inverted' for a second case (see below);
	actor_pose_shifted.Pos() = calculateBoxIntersection(object_anchor, actor_box);

	return (std::make_tuple(actor_pose_shifted, object_pose_shifted.Pos()));

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> Inflator::findIntersectedModelsClosestPoints(
		const ignition::math::Vector3d &actor_pos, const ignition::math::Vector3d &pt_intersect,
		const IntersectionType &type) const
{

	/*
	 * APPLIES ESPECIALLY to 2 actors case:
	 * While actors are moving towards each other the distance between their extreme points
	 * (those which intersect bounding boxes/circles) gets smaller. At some point it will
	 * become so small that 2 bounding boxes/circles will intersect. This creates some
	 * trouble connected with calculating the distance between closest points.
	 *
	 * The main issue there will be related with RelativeLocation of an object - it will likely
	 * switch from (for example) RIGHT to LEFT which in turn will create a switch in perpendicular
	 * vector direction too (RelativeLocation is calculated based on d_alpha_beta which relies on
	 * extreme points position - bingo!)
	 *
	 * This function prevents from accidental switch of Relative Location which produces sticking
	 * of 2 objects located near to each other. It will calculate the closest points (extreme)
	 * position as settled just around the half distance between 2 objects (their centers -
	 * to be specific).
	 */

#ifdef DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX_BB
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\nWARNING - " << SfmDebugGetCurrentActorName() << "'s bounding circle IS INTERSECTING " << SfmDebugGetCurrentObjectName() << "'s bounding BOX!" << std::endl;
	}
#endif

	// line from the actor's center to the point of intersection
	ignition::math::Line3d line_actor_intersection;
	line_actor_intersection.Set(ignition::math::Vector3d(actor_pos.X(),    actor_pos.Y(),    0.00),
			 	 	 	 	 	ignition::math::Vector3d(pt_intersect.X(), pt_intersect.Y(), 0.00));

	// calculate the slope of the created line
	ignition::math::Angle line_slope( std::atan2( line_actor_intersection.Direction().Y(), line_actor_intersection.Direction().X() ) );
	line_slope.Normalize(); // just in case

	/* locate the actor_shifted just around the intersection point (this will force
	 * a very short distance between the actor and the object) */
	ignition::math::Vector3d actor_shifted = actor_pos;


#ifdef DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX_BB
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\tLINE    slope: " << line_slope.Radian() << "\tdir: " << line_actor_intersection.Direction() << std::endl;
		std::cout << "\t\tlength: " << line_actor_intersection.Length() << "\tsin: " << sin(line_slope.Radian()) << "\tcos: " << cos(line_slope.Radian()) << std::endl;
		//std::cout << "\tINITIAL shift\tactor: " << actor_pose_shifted.Pos() << "\tobject: " <<  object_pos_shifted << std::endl;
		std::cout << "\tMODDED  shift\tactor: " << actor_shifted << "\tobject: " <<  _bb_intersection_pt << std::endl;
		std::cout << "\tDIST VECTOR: " << _bb_intersection_pt-actor_shifted << "\tlen: " << (_bb_intersection_pt-actor_shifted).Length() << std::endl;
		std::cout << std::endl;
	}
#endif

	// needed for 2 actors case
	ignition::math::Vector3d object_shifted = pt_intersect;

	// save length from actor center to intersection point (with bounding box)
	double length = line_actor_intersection.Length();

	// depending on `type` another repulsion distance will be considered
	// to maximize repulsion strength
	double max_repulsion_dist = 0.0;

	switch(type) {

	case(INTERSECTION_ACTORS):

			max_repulsion_dist = 0.1;
			/* Try to keep some threshold distance between intersected models,
			 * this is just a hack for a smoother SFM operation */
			if ( length > max_repulsion_dist ) {
				// keep distance long enough
				length = (length - max_repulsion_dist / length) / 2.0;
			} else {
				// unable to keep the distance long enough
				length *= 0.495;
			}

			/* Below is OK under assumption that both bounding `boxes` have the same shape and dimensions
			 * 1) 	create a line which divides the connection into 2 parts
			 * 2) 	re-assign shifted points - place then just around the center of the line
			 * 		to force very small distance between objects to strengthen their repulsion */
			actor_shifted.X( actor_pos.X() + length * cos(line_slope.Radian() ));
			actor_shifted.Y( actor_pos.Y() + length * sin(line_slope.Radian() ));
			object_shifted.X( pt_intersect.X() - length * cos(line_slope.Radian() ));
			object_shifted.Y( pt_intersect.Y() - length * sin(line_slope.Radian() ));
			return (std::make_tuple(actor_shifted, object_shifted));
			break;

	case(INTERSECTION_ACTOR_OBJECT):

			/* initially a factor was 0.97 but the smaller the distance between actor and an obstacle
			 * the smaller repulsion is produced; when the repulsion in small distances from an obstacle
			 * is too weak then the factor should be a little smaller  */

			/* NOTE1: `line_actor_intersection` is a vector connecting actor center position
			 * and object's extreme point.
			 * NOTE2: the `actor_shifted` vector is related to a point within actor bounding.
			 * A vector generated by a difference between this point and object's extreme point
			 * creates a new d_alpha_beta vector (effect of the `inflation` procedure).
			 * NOTE3: the bigger the coefficient near `line_actor_intersection.Length()`
			 * the shorter vector (d_alpha_beta) is generated.
			 * NOTE4: In general, a shorter vector not necessarily provides a stronger repulsion.
			 * The coefficient near `line_actor_intersection.Length()` is set to provide the strongest
			 * repulsion possible when models intersecting slightly.
			 * Exemplary LOG (last value is the coefficient):
			 * 	chair_1_clone: 	21.273 110.973 0	len: 112.993	dist: 0.0799756	model_type: 3	0.90
			 *	chair_1_clone: 	89.7493 505.829 0	len: 513.729	dist: 0.319737	model_type: 3	0.60
			 *	chair_1_clone: 	83.1285 496.072 0	len: 502.989	dist: 0.479551	model_type: 3	0.45
			 *	chair_1_clone: 	82.5894 365.857 0	len: 375.064	dist: 0.199734	model_type: 3	0.75
			 *	chair_1_clone: 	89.3611 495.877 0	len: 503.865	dist: 0.30368	model_type: 3	0.62
			 *	chair_1_clone: 	104.151 517.634 0	len: 528.008	dist: 0.35971	model_type: 3	0.55
			 *	chair_1_clone: 	88.3657 522.637 0	len: 530.055	dist: 0.391876	model_type: 3	0.51
			 * This applies to the circular bounding with radius of 0.8 m. For different size the coefficient
			 * will not be optimal but still should work properly.
			 */

			// based on logged data (see above) a distance with a strongest repulsion
			// is maintained (if possible).
			// NOTE: this `case` is connected with another way of interaction calculation
			// thus `length` is different than in the previous `case`
			//
			// FIXME: Experimental version, aim is to generate the strongest repulsion
			// possible when even slightly stepped into an obstacle.
			//
			max_repulsion_dist = 0.3; // 0.4

			if ( length > max_repulsion_dist ) {
				// keep distance long enough
				length = (length - max_repulsion_dist / length) / 2.0;
				actor_shifted.X( actor_pos.X() + length * cos(line_slope.Radian() ));
				actor_shifted.Y( actor_pos.Y() + length * sin(line_slope.Radian() ));
			} else {
				// Unable to keep the distance long enough.
				// Calculate how far should be shifted to maintain given distance.
				length = max_repulsion_dist - length;
				// Shift actor outwards object position (see sign below).
				actor_shifted.X( actor_pos.X() - length * cos(line_slope.Radian() ));
				actor_shifted.Y( actor_pos.Y() - length * sin(line_slope.Radian() ));
			}

//			// FIXME: just debugging
//			std::cout << "\tINTERSECTION_ACTOR_OBJECT | actor_pos: " << actor_pos << "\t\tobject_edge_pos: " << pt_intersect << std::endl;
//			std::cout << "\tINTERSECTION_ACTOR_OBJECT | actor_center-object_edge len: " << line_actor_intersection.Length() << "\t0.1x: " << 0.1 * line_actor_intersection.Length() << "\t\t0.9x: " << 0.9 * line_actor_intersection.Length() << std::endl;
//			std::cout << "\tINTERSECTION_ACTOR_OBJECT | actor_shifted-object_edge len: " << (pt_intersect - actor_shifted).Length() << std::endl;
//			std::cout << "\tINTERSECTION_ACTOR_OBJECT | actor_shifted: " << actor_shifted << std::endl;
			return (std::make_tuple(actor_shifted, pt_intersect));
			break;

	default:
		// this should never happen!
		return (std::make_tuple(actor_shifted, pt_intersect));

	}

}

// ------------------------------------------------------------------- //

std::tuple<bool, ignition::math::Vector3d> Inflator::isWithinRangeOfBB(
		const ignition::math::Vector3d &actor_center,
		const actor::inflation::Box &object_bb) const
{

	/* Using the fact that in Gazebo, the Bounding Box is axis-aligned
	 * so it is never rotated around the world Z axis.
	 * NOTE: actor's both X and Y coordinates will be within the BB range
	 * only if actor's center is located within object bounds. */

	// stores the end point of the `intersection line`
	ignition::math::Vector3d intersection_end = object_bb.getCenter();

	bool within_x = false;
	if ( actor_center.X() >= object_bb.getMin().X() && actor_center.X() <= object_bb.getMax().X() ) {
		within_x = true;
		intersection_end.X(actor_center.X());
	}

	bool within_y = false;
	if ( actor_center.Y() >= object_bb.getMin().Y() && actor_center.Y() <= object_bb.getMax().Y() ) {
		within_y = true;
		intersection_end.Y(actor_center.Y());
	}

	if ( (within_x && within_y) || (!within_x && !within_y) ) {
		// actor stepped onto an obstacle,
		// a procedure for this case is already implemented;
		// return unmodified center position of the object's BB
		return (std::make_tuple(false, intersection_end));
	}

	// some gap between actor and object is present - desired configuration
	return (std::make_tuple(true, intersection_end));

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Inflator::calculateBoxIntersection(const ignition::math::Vector3d &object_pos,
		const actor::inflation::Border &box,
		const ignition::math::Vector3d &box_pos) const {

	/* The algorithm of searching the closest points of 2 bounding boxes (presented below)
	 * consists of creation of a line starting in the 1st bounding box's center,
	 * ending in the 2nd bb's center; then the Intersects method of the Box class
	 * will be used to find 2 points creating the shortest distance between boxes */

	/* no error handling as there should always be an intersection found
	 * when 2 center points from each bounding box are connected by a line */

	/* the only exception is calculating the social force for visualization -
	 * in such a case there is a very high possibility of being unable to
	 * finding intersection */

	// will be likely updated later (return value)
	ignition::math::Vector3d box_intersection(box.getCenter().X(), box.getCenter().Y(), 0.0);

	// by default use box center position as line end point
	ignition::math::Vector3d box_pt = box.getCenter();

	// check if box_pos is valid number or NaN (default, see header file)
	if ( !std::isnan(box_pos.X()) ) {
		if ( box.doesContain(box_pos) ) {
			box_pt = box_pos;
		}
	}

	/* create a line from the object center to the box position and check
	 * the intersection point of that line with the object's bounding box */

	/* NOTE1: some models have their centers not matched with bounding box'es center
	 * which may produce false `intersects` flag. Let's stick to bounding
	 * box'es center (instead of `object_pose` */

	/* NOTE2: line direction is important. The line given by `A` point (start)
	 * and `B` point (end point) must cross the BoundingBox so the end
	 * point is located within BB bounds (direction INTO the box). */
	ignition::math::Line3d line;
	line.Set(object_pos.X(), object_pos.Y(), box_pt.X(), box_pt.Y(), box_pt.Z() );

	// find intersection point
	bool intersects = false;
	ignition::math::Vector3d point_intersect;
	std::tie(intersects, point_intersect) = box.doesIntersect(line);

	// check if intersection point was found - it always should
	// if `object_pos` is not located within BB bounds
	if ( intersects ) {
		box_intersection = point_intersect;
	}

	return (box_intersection);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Inflator::findClosestBoundingBoxVertex(const ignition::math::Vector3d &actor_pos,
		const actor::inflation::Border &object_box) const {

	std::vector<ignition::math::Vector3d> vertices_vector = createVerticesVector(object_box.getBox());
	std::vector<double> lengths_vector = calculateLengthToVertices(actor_pos, vertices_vector);
	auto shortest = std::min_element(lengths_vector.begin(), lengths_vector.end());

	if ( shortest == lengths_vector.end() ) {
		std::cout << "[ERROR] sfm::core::Inflator::findClosestBoundingBoxVertex" << std::endl;
	}

	// convert iterator to vector index
	size_t index = std::distance(lengths_vector.begin(), shortest);
	return (vertices_vector.at(index));

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findClosestPointsCircles(
		const ignition::math::Pose3d &actor_pose,
		const actor::inflation::Border &actor_circle,
		const ignition::math::Pose3d &object_pose,
		const actor::inflation::Border &object_circle,
		const std::string &_object_name /* debug only */) const
{

	/* BoundingCircle and BoundingCircle -> 2 actors */
	// intersection of the 1st actor's circle (currently processed)
	ignition::math::Pose3d actor_pose_shifted = actor_pose;
	std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_circle.doesIntersect(object_pose.Pos());

	// intersection of the 2nd actor's circle (another one)
	ignition::math::Vector3d object_pos_shifted;
	std::tie(std::ignore, object_pos_shifted) = object_circle.doesIntersect(actor_pose.Pos());

	/* Let's check whether the bounding circles are not intruding each other -
	 * compare the length of a vector from actor's center to the object's position
	 * shifted. When it's longer than radius then both bounding circles are safe -
	 * they are not intersecting each other. */
	if ( actor_circle.doesContain(object_pos_shifted) ) {
		std::tie(actor_pose_shifted.Pos(), object_pos_shifted) = findIntersectedModelsClosestPoints(actor_pose.Pos(), object_pose.Pos(), INTERSECTION_ACTORS);
	}

	return ( std::make_tuple(actor_pose_shifted, object_pos_shifted) );

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findClosestPointsEllipses(
		const ignition::math::Pose3d &actor_pose,
		const actor::inflation::Border &actor_ellipse,
		const ignition::math::Pose3d &object_pose,
		const actor::inflation::Border &object_ellipse,
		const std::string &object_name /* debug only */ ) const
{

	/* BoundingEllipse and BoundingEllipse -> 2 actors */
	// intersection of the 1st actor's circle (currently processed)
	ignition::math::Pose3d actor_pose_shifted = actor_pose;

	std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_ellipse.doesIntersect(object_pose.Pos());

	// intersection of the 2nd actor's circle (another one)
	ignition::math::Vector3d object_pos_shifted;

	// current actor ellipse's shifted center ( = actor's pos) is connected with object ellipse's center; the connector is a line
	std::tie(std::ignore, object_pos_shifted) = object_ellipse.doesIntersect(actor_pose.Pos());

	/* Let's check whether the bounding circles are not intruding each other -
	 * compare the length of a vector from actor's center to the object's position
	 * shifted. When it's longer than radius then both bounding circles are safe -
	 * they are not intersecting each other. */

	if ( actor_ellipse.doesContain(object_pos_shifted) ) {
		std::tie(actor_pose_shifted.Pos(), object_pos_shifted) = findIntersectedModelsClosestPoints(actor_pose.Pos(), object_pose.Pos(), INTERSECTION_ACTORS);
	}

	return ( std::make_tuple(actor_pose_shifted, object_pos_shifted) );

}

// ------------------------------------------------------------------- //

Inflator::~Inflator() { }

// ------------------------------------------------------------------- //

} /* namespace sfm */
