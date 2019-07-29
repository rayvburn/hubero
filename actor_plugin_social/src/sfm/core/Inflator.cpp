/*
 * Inflator.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#include <sfm/core/Inflator.h>

namespace sfm {
namespace core {

// ------------------------------------------------------------------- //

Inflator::Inflator() { }

// ------------------------------------------------------------------- //

ignition::math::Vector3d Inflator::findModelsClosestPoints(const ignition::math::Pose3d &actor_pose,
		const ignition::math::Pose3d &object_pose, const actor::inflation::Box &bb) const
{

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

	#ifdef DEBUG_BOUNDING_BOX
	if ( print_info ) {
		std::cout << "GetModelPointClosestToActor()" << "\tname: " << _model_name << "\tcenter: " << bb.Center() << "\tmax: " << bb.Max() << "\tmin: " << bb.Min() << "\n\t\t\t";
	}
	#endif

	// std::cout << "\nisinf: " << std::isinf( bb.Center().X() ) << "\tcenter_x: " << bb.Center().X() << std::endl;

	/* */
	// inf has an object with no bounding box defined (for example - actor)
	if ( std::fabs(bb.getCenter().X()) > 1e+300 ) {
		#ifdef DEBUG_BOUNDING_BOX
		if ( print_info ) {
			std::cout << "\tANOTHER ACTOR HERE!\n";
		}
		#endif
		return ( ignition::math::Vector3d(object_pose.Pos()) );
	}


	ignition::math::Line3d line;

	// Intersect() method returns a tuple
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
	// create a line of which intersection with a bounding box will be checked, syntax: x1, y1, x2, y2, z_common
//	line.Set(-1e+50, actor_pose.Pos().Y(), +1e+50, actor_pose.Pos().Y(), bb.Center().Z() );
	line.Set(actor_pose.Pos().X(), actor_pose.Pos().Y(), bb.getCenter().X(), actor_pose.Pos().Y(), bb.getCenter().Z() );
	std::tie(does_intersect, point_intersect) = bb.doesIntersect(line);

	if ( does_intersect ) {

		#ifdef DEBUG_BOUNDING_BOX
		if ( print_info ) {
			std::cout << "\tY-intersection - bounding box point: " << point_intersect << std::endl;
		}
		#endif

		return (point_intersect);

	}

	// 2nd case -------------------------------------------------------------------
//	line.Set(actor_pose.Pos().X(), -1e+50, actor_pose.Pos().X(), +1e+50, bb.Center().Z() );
	line.Set(actor_pose.Pos().X(), actor_pose.Pos().Y(), actor_pose.Pos().X(), bb.getCenter().Y(), bb.getCenter().Z() );
	std::tie(does_intersect, point_intersect) = bb.doesIntersect(line);

	if ( does_intersect ) {

		#ifdef DEBUG_BOUNDING_BOX
		if ( print_info ) {
			std::cout << "\tX-intersection - bounding box point: " << point_intersect << std::endl;
		}
		#endif

		return (point_intersect);

	}


	/* */
	// 3rd case -------------------------------------------------------------------
	std::vector<ignition::math::Vector3d> vertices_vector = createVerticesVector(bb);
	std::vector<double> lengths_vector = calculateLengthToVertices(actor_pose.Pos(), vertices_vector);

	double min_value = 3.4e+38;
	unsigned int index = 0;

	for ( size_t i = 0; i < lengths_vector.size(); i++ ) {

		if ( lengths_vector[i] < min_value ) {
			index = i;
			min_value = lengths_vector[i];
		}

	}

	#ifdef DEBUG_BOUNDING_BOX
	if ( print_info ) {
		std::cout << "\tvertices_vector: 0) " << vertices_vector[0] << "  1) " << vertices_vector[1] << "  2) " << vertices_vector[2] << "  3) " << vertices_vector[3];
		std::cout << "\n\t\t\t";
		std::cout << "\tlengths_vector: 0) " << lengths_vector[0] << "  1) " << lengths_vector[1] << "  2) " << lengths_vector[2] << "  3) " << lengths_vector[3];
		std::cout << "\n\t\t\t";
		std::cout << "\tCLOSEST VERTEX - bounding box point: " << vertices_vector[index] << std::endl;
	}
	#endif

	return (vertices_vector[index]);

}

// ------------------------------------------------------------------- //

std::vector<ignition::math::Vector3d> Inflator::createVerticesVector(const actor::inflation::Box &bb) const
{
	// 4 vertices only (on-plane)
	std::vector<ignition::math::Vector3d> temp_container;
	ignition::math::Vector3d temp_vector;

	temp_vector.Z(0.5); // hard-coded, planar objects considered

	temp_vector.X(bb.getMin().X()); 	temp_vector.Y(bb.getMin().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.getMin().X()); 	temp_vector.Y(bb.getMax().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.getMax().X()); 	temp_vector.Y(bb.getMin().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.getMax().X()); 	temp_vector.Y(bb.getMax().Y());
	temp_container.push_back(temp_vector);

	/* // emplace_back() blocks stdio messages
	temp_container.emplace_back(_bb.Min().X(), _bb.Min().Y(), BOUNDING_BOX_Z_FIXED);
	temp_container.emplace_back(_bb.Min().X(), _bb.Max().Y(), BOUNDING_BOX_Z_FIXED);
	temp_container.emplace_back(_bb.Max().X(), _bb.Min().Y(), BOUNDING_BOX_Z_FIXED);
	temp_container.emplace_back(_bb.Max().X(), _bb.Max().Y(), BOUNDING_BOX_Z_FIXED);
	*/

	return (temp_container);

}

// ------------------------------------------------------------------- //

std::vector<double> Inflator::calculateLengthToVertices(const ignition::math::Vector3d &actor_pos,
		const std::vector<ignition::math::Vector3d> &vertices_pts) const
{

	std::vector<double> temp_containter;
	for ( size_t i = 0; i < vertices_pts.size(); i++ ) {
		temp_containter.push_back( (vertices_pts[i] - actor_pos).Length() );
	}
	return (temp_containter);

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findModelsClosestPoints(
		const ignition::math::Pose3d &actor_pose,  const actor::inflation::Box &actor_box,
		const ignition::math::Pose3d &object_pose, const actor::inflation::Box &object_box,
		const std::string &_object_name /* debug only */ ) const
{

	/* the below algorithm of searching the closest points of 2 bounding boxes
	 * consists of creation of a line starting in the 1st bounding box's center,
	 * ending in the 2nd bb's center; then the Intersects method of the Box class
	 * will be used to find 2 points creating the shortest distance between boxes */

	/* no error handling as there should always be an intersection found
	 * when 2 center points from each bounding box are connected by a line */

	/* the only exception is calculating the social force for visualization -
	 * in such a case there is a very high possibility of being unable to
	 * finding intersection */

	// for some reason the line has to be 'inverted' for a second case (see below)

	ignition::math::Line3d line;

	// Intersect() method returns a tuple
	bool does_intersect = false;
	double dist_intersect = 0.0;
	ignition::math::Vector3d point_intersect;

	// actor's bounding box point that is closest to object's bounding box
	line.Set(object_box.getCenter().X(), object_box.getCenter().Y(), actor_pose.Pos().X(), actor_pose.Pos().Y(), actor_box.getCenter().Z() );
	std::tie(does_intersect, point_intersect) = actor_box.doesIntersect(line);

	// create new pose based on actor's pose, POSSIBLY update only position component
	ignition::math::Pose3d actor_pose_shifted = actor_pose;

	if ( !does_intersect ) {
		#ifdef DEBUG_BOUNDING_BOX
		std::cout << "\n\n\nGetActorModelBBsClosestPoints() ERROR1"; // \n\n\n";
		std::cout << "\nACTOR BB check - center: " << _actor_bb.Center() << "\tmin: " << _actor_bb.Min() << "\tmax: " << _actor_bb.Max() << std::endl;
		std::cout << "ACTOR pos: " << _actor_pose.Pos().X() << " " << _actor_pose.Pos().Y() << "  BB closest: " << point_intersect.X() << " " << point_intersect.Y() << std::endl;
		std::cout << "OBJECT pos: " << _object_pose.Pos();
		std::cout << "\n\n\n";
		#endif
	} else {
		// the intersection is found - update
		actor_pose_shifted.Pos() = point_intersect;
	}


	// object's bounding box point that is closest to actor's bounding box
	line.Set(actor_pose.Pos().X(), actor_pose.Pos().Y(), object_box.getCenter().X(), object_box.getCenter().Y(), object_box.getCenter().Z() );
	std::tie(does_intersect, point_intersect) = object_box.doesIntersect(line);

	if ( !does_intersect ) {
		#ifdef DEBUG_BOUNDING_BOX
		std::cout << "\n\n\nGetActorModelBBsClosestPoints() ERROR2"; // \n\n\n";
		std::cout << "\t" << _object_name << "'s pos: " << _object_pose.Pos() << "\tBB closest: " << point_intersect.X() << " " << point_intersect.Y() << std::endl;
		std::cout << "\n\n\n";
		#endif
		// the intersection was not found - set intersection point as a object's central point
		point_intersect = object_pose.Pos();
	} else {
		// that's ok, point_intersect will be returned from function
	}

	return ( std::make_tuple(actor_pose_shifted, point_intersect) );

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
	double length = 0.0;

	switch(type) {

	case(INTERSECTION_ACTORS):
			/* Try to keep some threshold distance between intersected models,
			 * this is just a hack for a smoother SFM operation */
			length = line_actor_intersection.Length();
			if ( length > 0.1 ) {
				// keep distance long enough
				length = (length - 0.1 / length) / 2.0;
			} else {
				// unable to keep the distance long enough
				length = 0.495 * line_actor_intersection.Length();
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
			actor_shifted.X( actor_pos.X() + 0.4 * line_actor_intersection.Length() * cos(line_slope.Radian() ));
			actor_shifted.Y( actor_pos.Y() + 0.4 * line_actor_intersection.Length() * sin(line_slope.Radian() ));
			return (std::make_tuple(actor_shifted, pt_intersect));
			break;

	default:
		// this should never happen!
		return (std::make_tuple(actor_shifted, pt_intersect));

	}

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findModelsClosestPoints(
		const ignition::math::Pose3d &actor_pose,
		const actor::inflation::Circle &actor_circle,
		const ignition::math::Pose3d &object_pose,
		const actor::inflation::Circle &object_circle,
		const std::string &_object_name /* debug only */) const
{

	/* BoundingCircle and BoundingCircle -> 2 actors */
	// intersection of the 1st actor's circle (currently processed)
	ignition::math::Pose3d actor_pose_shifted = actor_pose;
	std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_circle.getIntersection(object_pose.Pos());

	// intersection of the 2nd actor's circle (another one)
	ignition::math::Vector3d object_pos_shifted;
	std::tie(std::ignore, object_pos_shifted) = object_circle.getIntersection(actor_pose.Pos());

	/* Let's check whether the bounding circles are not intruding each other -
	 * compare the length of a vector from actor's center to the object's position
	 * shifted. When it's longer than radius then both bounding circles are safe -
	 * they are not intersecting each other. */
//	ignition::math::Vector3d connection;
//	connection = object_pos_shifted - actor_pose.Pos();
//	connection.Z(0.0); // planar
//
//	if ( connection.Length() <= actor_circle.GetRadius() ) {
//	std::cout << SfmDebugGetCurrentActorName() << "'s center: " << actor_pose.Pos() << "\tBC's center: " << actor_circle.GetCenter() << std::endl;

	// above commented -> instead isWithin method created
	if ( actor_circle.doesContain(object_pos_shifted) ) {

		std::tie(actor_pose_shifted.Pos(), object_pos_shifted) = findIntersectedModelsClosestPoints(actor_pose.Pos(), object_pose.Pos(), INTERSECTION_ACTORS);

	} else {

		#ifdef DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX
		std::cout << "\nINFO - " << SfmDebugGetCurrentActorName() << "'s bounding circle IS SAFE\tlength: " << connection.Length() << "\tradius: " << actor_circle.GetRadius() << std::endl;
		#endif

	}

	#ifdef DEBUG_BOUNDING_CIRCLE
	std::cout << "\n\nBOUND - 2 actors | 1 pos: " << actor_pose.Pos() << "\tintersection: " << actor_pose_shifted.Pos() << std::endl;
	std::cout << "BOUND - 2 actors |" << _object_name << "'s pos: " << object_pose.Pos() << "\tintersection: " << object_pos_shifted << std::endl;
	#endif

	return ( std::make_tuple(actor_pose_shifted, object_pos_shifted) );

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findModelsClosestPoints(
		const ignition::math::Pose3d &actor_pose,
		const actor::inflation::Ellipse &actor_ellipse,
		const ignition::math::Pose3d &object_pose,
		const actor::inflation::Ellipse &object_ellipse,
		const std::string &object_name /* debug only */ ) const
{

	/* BoundingEllipse and BoundingEllipse -> 2 actors */
	// intersection of the 1st actor's circle (currently processed)
	ignition::math::Pose3d actor_pose_shifted = actor_pose;

	#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmGetPrintData() ) {
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\n*******************************************************************************\n";
		std::cout << "in GetActorModelBBsClosestPoints() - 2 ACTORS - checking intersection\n";
		std::cout << "\t" << SfmDebugGetCurrentActorName() << "'s pos: " << actor_pose.Pos() << "\t" << SfmDebugGetCurrentObjectName() << "'s pos: " << object_pose.Pos() << std::endl;
	}
	}
	#endif

	std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_ellipse.getIntersection(object_pose.Pos());

	#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmGetPrintData() ) {
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\n\tActor's BE intersection result: " << actor_pose_shifted.Pos();
	}
	}
	#endif

	// intersection of the 2nd actor's circle (another one)
	ignition::math::Vector3d object_pos_shifted;
	// current actor ellipse's shifted center ( = actor's pos) is connected with object ellipse's center; the connector is a line
	std::tie(std::ignore, object_pos_shifted) = object_ellipse.getIntersection(actor_pose.Pos());
//	std::tie(std::ignore, object_pos_shifted) = object_ellipse.getIntersection(actor_pose_shifted.Pos());

	#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmGetPrintData() ) {
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\n\tObject's BE intersection result: " << object_pos_shifted;
	}
	}
	#endif

	/* Let's check whether the bounding circles are not intruding each other -
	 * compare the length of a vector from actor's center to the object's position
	 * shifted. When it's longer than radius then both bounding circles are safe -
	 * they are not intersecting each other. */

	if ( actor_ellipse.doesContain(object_pos_shifted) ) {

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmGetPrintData() ) {
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
			std::cout << "\n\n\tACTOR'S BE CONTAINS " << SfmDebugGetCurrentObjectName() << "'s POINT!!!!\n";
		}
		}
		#endif
		std::tie(actor_pose_shifted.Pos(), object_pos_shifted) = findIntersectedModelsClosestPoints(actor_pose.Pos(), object_pose.Pos(), INTERSECTION_ACTORS);

	} else {

		#ifdef DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX
		std::cout << "\nINFO - " << SfmDebugGetCurrentActorName() << "'s bounding circle IS SAFE\tlength: " << connection.Length() << "\tradius: " << _actor_bc.GetRadius() << std::endl;
		#endif

	}

	#ifdef DEBUG_BOUNDING_CIRCLE
	std::cout << "\n\nBOUND - 2 actors | 1 pos: " << actor_pose.Pos() << "\tintersection: " << actor_pose_shifted.Pos() << std::endl;
	std::cout << "BOUND - 2 actors |" << _object_name << "'s pos: " << object_pose.Pos() << "\tintersection: " << object_pos_shifted << std::endl;
	#endif

	#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmGetPrintData() ) {
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\n*******************************************************************************\n\n\n";
	}
	}
	#endif

	return ( std::make_tuple(actor_pose_shifted, object_pos_shifted) );

}

// ------------------------------------------------------------------- //

Inflator::~Inflator() { }

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace sfm */
