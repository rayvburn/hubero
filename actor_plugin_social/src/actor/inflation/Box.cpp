/*
 * Box.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */


#include <actor/inflation/Box.h>
#include <math.h>
#include <iostream> // debugging
#include "actor/FrameGlobal.h" // global frame id

namespace actor {
namespace inflation {

// ------------------------------------------------------------------- //

Box::Box():	bb_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
			bb_x_half_(0.45), bb_y_half_(0.45), bb_z_half_(1.00)
{
	// protected member of the base class
	this->is_box_ = true;
}

// ------------------------------------------------------------------- //

// conversion
Box::Box(const ignition::math::Box &bb):
		bb_(bb),bb_x_half_(0.45), bb_y_half_(0.45), bb_z_half_(1.00)
{
}

// ------------------------------------------------------------------- //

void Box::init(const double &x_half_len, const double &y_half_len, const double &z_half_len) {
	bb_x_half_ = x_half_len;
	bb_y_half_ = y_half_len;
	bb_z_half_ = z_half_len;
}

// ------------------------------------------------------------------- //

void Box::setBox(const ignition::math::Box &bb) {
	bb_ = bb;
}

// ------------------------------------------------------------------- //

void Box::updatePose(const ignition::math::Pose3d &new_pose) {

	// forced calculations in 0-90 deg range
	double yaw_truncated = std::fabs(new_pose.Rot().Yaw());
	yaw_truncated -= static_cast<int>(yaw_truncated / IGN_PI_2) * IGN_PI_2;

	/* TODO: Add roll and pitch rotation handling (as actor lies down X and Z changes);
	 * 		 it won't be used in move_around state but at last will be nice
	 * 		 to have such a feature */

//	double roll_truncated = std::fabs(_actor_pose.Rot().Roll());
//	roll_truncated -= static_cast<int>(roll_truncated / IGN_PI_2) * IGN_PI_2;
//
//	double pitch_truncated = std::fabs(_actor_pose.Rot().Pitch());
//	pitch_truncated -= static_cast<int>(pitch_truncated / IGN_PI_2) * IGN_PI_2;

	// x-projection on x-axis
	double xp = bb_x_half_ * cos(yaw_truncated);
	// line projected on x-axis that extends basic x-projection
	double xp_ext = bb_y_half_ * sin(yaw_truncated);
	// sum of xp's gives total length of x-component of BB
	double x_total = xp + xp_ext;

	// the same for y-axis
	double yp = bb_y_half_ * cos(yaw_truncated);
	double yp_ext = bb_x_half_ * sin(yaw_truncated);
	double y_total = yp + yp_ext;

	ignition::math::Box bb(	new_pose.Pos().X() - x_total,
							new_pose.Pos().Y() - y_total,
							new_pose.Pos().Z() - bb_z_half_,
							new_pose.Pos().X() + x_total,
							new_pose.Pos().Y() + y_total,
							new_pose.Pos().Z() + bb_z_half_ );

	// debugging (half -> 0.7?)
//	if ( (bb.Max().X() - bb.Min().X()) < 0.90 || (bb.Max().X() - bb.Min().X()) > 1.272793 ) {
//		std::cout << "\tBB min: " << bb.Min() << "\tmax: " << bb.Max() << std::endl;
//		std::cout << "\t\tcenter: " << bb.Center() << "\tRAW x: " << new_pose.Pos().X() - bb_x_half_ << "\tRAW y: " << new_pose.Pos().Y() + bb_y_half_ << std::endl;
//		std::cout << "\t\tyaw: " << new_pose.Rot().Yaw() << std::endl;
//		std::cout << "\t\txp: " << xp << "\txp_ext: " << xp_ext << "\tx_total: " << x_total << std::endl;
//		std::cout << "\t\typ: " << yp << "\typ_ext: " << yp_ext << "\ty_total: " << y_total << std::endl;
//		std::cout << "\t\tlen: " << bb.Max().X() - bb.Min().X() << std::endl;
//		std::cout << "\t\tWRONG LENGTH\n\n" << std::endl;
//		std::cout << std::endl;
//		std::cout << std::endl;
//		std::cout << std::endl;
//	}

	bb_ = bb;

}

// ------------------------------------------------------------------- //

bool Box::doesContain(const ignition::math::Vector3d &pt) const {
	return ( bb_.Contains(pt) );
}

// ------------------------------------------------------------------- //

std::tuple<bool, ignition::math::Vector3d> Box::doesIntersect(const ignition::math::Line3d &line) const {
	bool does_intersect = false;
	ignition::math::Vector3d pt_intersection;
	std::tie(does_intersect, std::ignore, pt_intersection) = bb_.Intersect(line);
	return ( std::make_tuple(does_intersect, pt_intersection) );
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Box::getMarkerConversion() const {

	visualization_msgs::Marker marker;

	marker.header.frame_id = actor::FrameGlobal::getFrame();
	marker.header.stamp = ros::Time();
	marker.ns = "test";
	// marker.id = marker_arr_index;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::MODIFY;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = bb_.Center().X();
	marker.pose.position.y = bb_.Center().Y();
	marker.pose.position.z = bb_.Center().Z();

	marker.pose.orientation.x = 0.00;
	marker.pose.orientation.y = 0.00;
	marker.pose.orientation.z = 0.00;
	marker.pose.orientation.w = 1.00;

	// scale
	marker.scale.x = std::fabs(bb_.Max().X() - bb_.Min().X());
	marker.scale.y = std::fabs(bb_.Max().Y() - bb_.Min().Y());
	marker.scale.z = std::fabs(bb_.Max().Z() - bb_.Min().Z());

	marker.color.a = 0.5; // alpha channel
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	return (marker);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Box::getCenter() 	const { return ( bb_.Center() ); }
ignition::math::Vector3d Box::getMin() 		const { return ( bb_.Min() ); }
ignition::math::Vector3d Box::getMax() 		const { return ( bb_.Max() ); }
ignition::math::Box 	 Box::getBox() 		const { return ( bb_ ); };

// ------------------------------------------------------------------- //

Box::~Box() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

} /* namespace inflation */
} /* namespace actor */
