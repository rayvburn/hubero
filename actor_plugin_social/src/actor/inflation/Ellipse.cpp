/*
 * Ellipse.cpp
 *
 *  Created on: Mar 31, 2019
 *      Author: rayvburn
 */

#include <actor/inflation/Ellipse.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Quaternion.hh>
#include <math.h> // fabs(), pow(), atan2(), tan()
#include <sfm/core/SFMDebug.h>
#include "actor/FrameGlobal.h" // global frame id

namespace actor {
namespace inflation {

// ------------------------------------------------------------------- //

Ellipse::Ellipse(): a_major_(1.2), b_minor_(0.5), yaw_ellipse_(0.0), yaw_offset_(0.0) { }

// ------------------------------------------------------------------- //

Ellipse::Ellipse(const double &a_major, const double &b_minor, const double &yaw,
								 const ignition::math::Vector3d &center_point,
								 const ignition::math::Vector3d &offset_vector) {

	init(a_major, b_minor, yaw, center_point, offset_vector);
}

// ------------------------------------------------------------------- //

void Ellipse::init(const double &a_major, const double &b_minor, const double &yaw,
		  const ignition::math::Vector3d &center_point,
		  const ignition::math::Vector3d &offset_vector) {

	setSemiMajorAxis(a_major);
	setSemiMinorAxis(b_minor);
	setCenterOffset(offset_vector);
	updatePose(ignition::math::Pose3d(center_point, ignition::math::Quaterniond(0.0, 0.0, yaw)));

}

// ------------------------------------------------------------------- //

void Ellipse::setSemiMajorAxis(const double &a_major) {
	a_major_ = std::fabs(a_major);
}

// ------------------------------------------------------------------- //

void Ellipse::setSemiMinorAxis(const double &b_minor) {
	b_minor_ = std::fabs(b_minor);
}

// ------------------------------------------------------------------- //

void Ellipse::setYaw(const double &yaw_ellipse) {
	yaw_ellipse_ = yaw_ellipse;
	updateCenter();
}

// ------------------------------------------------------------------- //

void Ellipse::setPosition(const ignition::math::Vector3d &center_point) {
	center_shifted_ = center_point;
	updateCenter();
}

// ------------------------------------------------------------------- //

/// expressed in m along semi-axes
/// @offset_vector - positive X shifts towards actor's back
/// @offset_vector - positive Y shifts towards actor's right-hand side
void Ellipse::setCenterOffset(const ignition::math::Vector3d &offset_vector) {

	/* check if offset is lying down within the ellipse's bound -
	 * at this moment the offset is 0 and location of the offset point
	 * must be investigated */
	if ( offset_vector.Length() > 1e-06 && doesContain(center_ + offset_vector) ) { // FIXME - ROTATION not taken into consideration

		offset_ = offset_vector;

		// calculate yaw_offset based on the offset vector
		ignition::math::Angle yaw_offset_angle;
		yaw_offset_angle.Radian( std::atan2(offset_vector.Y(), offset_vector.X()) );
		yaw_offset_angle.Normalize();
		yaw_offset_ = yaw_offset_angle.Radian();
		std::cout << "Ellipse::setCenterOffset() set successfully!" << std::endl;

	} else {

		// TODO: some error message
		std::cout << "Ellipse::setCenterOffset() couldn't be set!" << std::endl;

	}

}

// ------------------------------------------------------------------- //

/// ellipse orientation must match world's coordinate system -
/// a conversion from actor's coordinate system must be done!
void Ellipse::updatePose(const ignition::math::Pose3d &pose) {
	center_shifted_ = pose.Pos();
	yaw_ellipse_ = pose.Rot().Yaw();
	updateCenter();
}

// ------------------------------------------------------------------- //

/// find point in which line going through ellipse's center and pt_dest intersects the ellipse
/// determine if pt_dest lies within ellipse's bounds

std::tuple<bool, ignition::math::Vector3d> Ellipse::getIntersection(const ignition::math::Vector3d &pt_dest) const {

//	/*
//	 * https://en.wikipedia.org/wiki/Ellipse#Shifted_Ellipse
//	 *
//	 * Using the sine and cosine functions a parametric representation
//	 * of the ellipse with shifted center is as follows:
//	 *
//	 * (c1 + a*cos(theta); c2 + b*sin(theta),
//	 * where 0 <= theta < 2pi
//	 *
//	 */
//

//	// V1
//	// 1st calculate the angle from center to the dest point
//	ignition::math::Angle theta;
//	ignition::math::Vector3d to_dest;
//	to_dest = pt_dest - (center_ + offset_);
//	theta.Radian(std::atan2(to_dest.Y(), to_dest.X()));
//	theta.Normalize();
//
//	// 2nd take the ellipse orientation into consideration
//	theta.Radian( theta.Radian() + yaw_ );
//	theta.Normalize();
//
//	// 3rd find the point of intersection of the to_dest vector and the ellipse
//	ignition::math::Vector3d pt_intersection;
//	pt_intersection.X( center_.X() + ( offset_.X() + (a_major_ * cos(theta.Radian())) ) );
//	pt_intersection.Y( center_.Y() + ( offset_.Y() + (b_minor_ * sin(theta.Radian())) ) );
//	pt_intersection.Z(0.0f);
//
//	return (pt_intersection);

	// lack of angle to check intersection from center (not shifted)
//	// V2
//	ignition::math::Vector3d pt_intersection;
//	double angle_center_dest;
//	std::tie(pt_intersection, angle_center_dest) = getIntersectionExtended(pt_dest);
//	return (pt_intersection);


	// V3
	// TODO: method similar to the ignition::math::Box's Intersects - returns a tuple:
	// Vector3d pt_of_intersection, bool is_within  ||  dist from center to intersection point

	/* calculate the angle from the ellipse's interior (its center if offset is 0)
	 * to the destination point */
	ignition::math::Angle psi;
	ignition::math::Vector3d to_dest = pt_dest - center_;
	psi.Radian(std::atan2(to_dest.Y(), to_dest.X()));
	psi.Normalize();

#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
	std::cout << "\n\nEllipse::getIntersection() with point: " << pt_dest << "\tellipse's center: " << center_ << "  center_shifted: " << center_shifted_ << std::endl;
	std::cout << "\t\t\tpsi_to_dest from center shifted: " << psi.Radian() << std::endl;
	}
#endif

	/* find line's points of intersection with ellipse;
	 * line is created based on psi value (from the shifted
	 * center to the destination point */

	// returns a 3-element tuple
	unsigned int solution_num = 0;
	ignition::math::Vector3d pt_of_intersection1, pt_of_intersection2;
	std::tie(solution_num, pt_of_intersection1, pt_of_intersection2) = getIntersectionWithLine(psi.Radian());

#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
	std::cout << "\tsolutions: " << solution_num << "\tintersection with p1: " << pt_of_intersection1 << "\tor p2: " << pt_of_intersection2 << std::endl;
	}
#endif

	/* check which intersection point is 'correct' based on psi */
	ignition::math::Vector3d *pt_proper;
	ignition::math::Angle angle_test( std::atan2( (pt_of_intersection1 - center_).Y(),
												  (pt_of_intersection1 - center_).X()) );
	angle_test.Normalize();


	/* correct angle will be nearly equal to 0, wrong point's vector
	 * will produce doubled angle */

#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	ignition::math::Angle angle_test_backup;
	angle_test_backup.Radian( std::atan2( (pt_of_intersection2 - center_).Y(),
								   	   	   (pt_of_intersection2 - center_).X() ) );
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
	std::cout << "\tangle_test_p1: " << angle_test.Radian() << "\tangle_test_p2: " << angle_test_backup.Radian() << "\tpsi: " << psi.Radian() << std::endl;
	}
#endif

	// TODO: some error handling? 1 solution?
	if ( std::fabs(angle_test.Radian() - psi.Radian()) < 1e-03 ) {
		pt_proper = &pt_of_intersection1;

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\tp1 - PASSED" << std::endl;
		}
		#endif

	} else {

		// debug ------------------------- -------------------------
		angle_test.Radian( std::atan2( (pt_of_intersection2 - center_).Y(),
									   (pt_of_intersection2 - center_).X() ) );
		if ( std::fabs(angle_test.Radian() - psi.Radian()) < 1e-03 ) {

			#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
			if ( SfmDebugGetCurrentActorName() == "actor1" ) {
			std::cout << "\tp2 - PASSED" << std::endl;
			}
			#endif

		} else {

			#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
			if ( SfmDebugGetCurrentActorName() == "actor1" ) {
			std::cout << "\tFAILED" << std::endl;
			}
			#endif

		}
		// ------------------------- -------------------------

		pt_proper = &pt_of_intersection2;
	}

#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
	std::cout << "\tproper point: " << *pt_proper << std::endl;
	std::cout << "\tpoint location test - pt_dest to center len: " << (pt_dest - center_).Length() << "\tpoint on ellipse to center len: " << (*pt_proper - center_).Length();
	}
#endif

	/* check if destination point lies within the ellipse */
	bool is_within = false;
	if ( (pt_dest - center_).Length() - (*pt_proper - center_).Length() <= 0.0 ) {

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "  WITHIN!" << std::endl;
		}
		#endif

		is_within = true;
	} else {

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "  PT is OUTSIDE ELLIPSE!" << std::endl;
		}
		#endif

	}

	return (std::make_tuple(is_within, *pt_proper));

}

// ------------------------------------------------------------------- //

bool Ellipse::doesContain(const ignition::math::Vector3d &pt) const {

	bool is_within;
	std::tie(is_within, std::ignore) = getIntersection(pt);
	return (is_within);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Ellipse::getCenter() const {
	return (center_);
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Ellipse::getCenterOffset() const {
	return (offset_);
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Ellipse::getCenterShifted() const {
	return (center_shifted_);
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Ellipse::getMarkerConversion() const {

	visualization_msgs::Marker marker;

	marker.header.frame_id = actor::FrameGlobal::getFrame();
	//marker.header.stamp = 0.0;
	marker.ns = "test"; // TODO: change ns name
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::MODIFY;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = center_.X();
	marker.pose.position.y = center_.Y();
	marker.pose.position.z = 0.9; // half of a typical person height

	// convert rpy to quaternion and set marker's orientation
	ignition::math::Quaterniond rot(0.0, 0.0, yaw_ellipse_);
	marker.pose.orientation.x = rot.X();
	marker.pose.orientation.y = rot.Y();
	marker.pose.orientation.z = rot.Z();
	marker.pose.orientation.w = rot.W();

	// scale
	marker.scale.x = 2.0 * a_major_;
	marker.scale.y = 2.0 * b_minor_;
	marker.scale.z = 1.8f; 	// typical person height

	marker.color.a = 0.5; 	// alpha channel
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	return (marker);

}

// ------------------------------------------------------------------- //

void Ellipse::updateShiftedCenter() {

	// DEPRECATED, updateCenter used as shifted center is equal to actor's position
	center_shifted_.X(center_.X() + offset_.X() * cos(yaw_offset_ + yaw_ellipse_));
	center_shifted_.Y(center_.Y() + offset_.Y() * sin(yaw_offset_ + yaw_ellipse_));
	center_shifted_.Z(center_.Z());

}

// ------------------------------------------------------------------- //

void Ellipse::updateCenter() {

	// center = center_shifted + rotated_offset (2D rotation matrix expanded here)
	center_.X( center_shifted_.X() + (offset_.X()*cos(yaw_ellipse_) - offset_.Y()*sin(yaw_ellipse_)) );
	center_.Y( center_shifted_.Y() + (offset_.X()*sin(yaw_ellipse_) + offset_.Y()*cos(yaw_ellipse_)) );
	center_.Z( 0.0 );

}

// ------------------------------------------------------------------- //

// useful for simple intersection from center to destination point search
std::tuple<ignition::math::Vector3d, double> Ellipse::getIntersectionExtended(const ignition::math::Vector3d &pt_dest)
	const {

	/*
	 * https://en.wikipedia.org/wiki/Ellipse#Shifted_Ellipse
	 *
	 * Using the sine and cosine functions a parametric representation
	 * of the ellipse with shifted center is as follows:
	 *
	 * (c1 + a*cos(theta); c2 + b*sin(theta),
	 * where 0 <= theta < 2pi
	 *
	 */

	/* 1st calculate the angle from the ellipse's interior (its center if offset is 0)
	 * to the destination point */
	ignition::math::Angle psi;
	ignition::math::Vector3d to_dest = pt_dest - center_shifted_;
	psi.Radian(std::atan2(to_dest.Y(), to_dest.X()));
	psi.Normalize();

	// 2nd take the ellipse orientation into consideration
	psi.Radian( psi.Radian() + yaw_ellipse_ );
	psi.Normalize();

	// 3rd find the point of intersection of the to_dest vector and the ellipse
	ignition::math::Vector3d pt_intersection;
	pt_intersection.X( center_.X() + ( offset_.X() + (a_major_ * cos(psi.Radian())) ) );
	pt_intersection.Y( center_.Y() + ( offset_.Y() + (b_minor_ * sin(psi.Radian())) ) );
	pt_intersection.Z(0.0f);

	return (std::make_tuple( pt_intersection, psi.Radian() ));

}

// ------------------------------------------------------------------- //

// useful for intersection from center to destination point search
/// return: number of solutions; 1st intersection point; 2nd intersection point
/// when 0 solutions found returns 0,0,0 vectors

std::tuple<unsigned int, ignition::math::Vector3d, ignition::math::Vector3d>
Ellipse::getIntersectionWithLine(const double &to_dest_angle) const {

#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
	std::cout << "\ngetIntersectionWithLine()\t\t\t\tSTART\n";
	}
#endif

	// number of solutions indicator
	unsigned int solution_num = 0;

	// slope of a line
	double a_l = std::tan(to_dest_angle);

	// intercept
	//double b_l = offset_.Y() + center_.X() * a_l;
	//double b_l = center_.Y() + center_.X() * a_l;
	// compute intercept knowing the line passes through the shifted center of the ellipse
	double b_l = center_.Y() - center_.X() * a_l;

	/* based on ellipse parametric equation, using quadratic equation,
	 * find intersection points of the line and the ellipse;
	 * calculations made with Matlab using symbolic variables -
	 * details could be found in documentation */

	// quadratic equation coefficients
	double a_q = std::pow( (cos(yaw_ellipse_) + a_l*sin(yaw_ellipse_)), 2.00) / std::pow(a_major_, 2.00) +
				 std::pow( (sin(yaw_ellipse_) - a_l*cos(yaw_ellipse_)), 2.00) / std::pow(b_minor_, 2.00);

	double b_q = -(2.0 * (center_.X()*cos(yaw_ellipse_) - sin(yaw_ellipse_) * (b_l - center_.Y())) * (cos(yaw_ellipse_) + a_l*sin(yaw_ellipse_))) / std::pow(a_major_, 2.00)
				 -(2.0 * (center_.X()*sin(yaw_ellipse_) + cos(yaw_ellipse_) * (b_l - center_.Y())) * (sin(yaw_ellipse_) - a_l*cos(yaw_ellipse_))) / std::pow(b_minor_, 2.00);

	double c_q = std::pow(( center_.X() * cos(yaw_ellipse_) - sin(yaw_ellipse_) * (b_l - center_.Y()) ), 2.00) / std::pow(a_major_, 2.00) +
				 std::pow(( center_.X() * sin(yaw_ellipse_) + cos(yaw_ellipse_) * (b_l - center_.Y()) ), 2.00) / std::pow(b_minor_, 2.00) - 1.00;

	double delta = std::pow(b_q, 2.00) - (4.00 * a_q * c_q);

#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
	std::cout << "\n\tLINE - a: " << a_l << "   b: " << b_l << "   delta: " << delta << "   to_dest_ang: " << to_dest_angle;
	} else {
		//std::cout << "\n\n\n\n\nELLIPSE TO NOT PRINT DEBUG\n\n\n\n\n";
	}
#endif

	// check number of solutions
	if ( std::fabs(delta) < 1e-06 ) {
		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\t1 solution\n";
		}
		#endif
		solution_num = 1;
	} else if ( delta > 0.0 ) {
		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\t2 solutions\n";
		}
		#endif

		solution_num = 2;
	} else {
		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\t0 solutions\n";
		}
		#endif
		solution_num = 0;
	}

	// calculate intersection point(s) based on solution_num
	if ( solution_num == 2 ) {

		ignition::math::Vector3d pt_of_intersection1, pt_of_intersection2;
		pt_of_intersection1.X( (-b_q - std::sqrt(delta)) / (2.00 * a_q) );
		pt_of_intersection1.Y( a_l * pt_of_intersection1.X() + b_l );

		pt_of_intersection2.X( (-b_q + std::sqrt(delta)) / (2.00 * a_q) );
		pt_of_intersection2.Y( a_l * pt_of_intersection2.X() + b_l );

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\n\tQUADRATIC EQUATION" << std::endl;
		std::cout << "\tparams: a_major: " << a_major_ << "  b_minor: " << b_minor_ << "  center_shifted: " << center_ << std::endl;
		std::cout << "\tdelta: " << delta << "  sqrt_delta: " << sqrt(delta) << "  a_q: " << a_q << "  b_q: " << b_q << "  c_q: " << c_q << std::endl;
		std::cout << "\n";
		std::cout << "\ngetIntersectionWithLine()\t\t\t\tEND\n";
		}
		#endif

		return ( std::make_tuple( 2, pt_of_intersection1, pt_of_intersection2 ) );

	} else if ( solution_num == 1 ) {

		ignition::math::Vector3d pt_of_intersection;
		pt_of_intersection.X( (-b_q) / (2.00 * a_q) );
		pt_of_intersection.Y( a_l * pt_of_intersection.X() + b_l );

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\ngetIntersectionWithLine() -------------------- END --------------------\n";
		}
		#endif

		return ( std::make_tuple( 1, pt_of_intersection, ignition::math::Vector3d() ) );

	} else {

		#ifdef DEBUG_BOUNDING_ELLIPSE_INTERSECTION
		if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\ngetIntersectionWithLine() -------------------- END --------------------\n";
		}
		#endif
		return ( std::make_tuple( 0, ignition::math::Vector3d(), ignition::math::Vector3d() ) );

	}


}

// ------------------------------------------------------------------- //

Ellipse::~Ellipse() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace inflation */
} /* namespace actor */
