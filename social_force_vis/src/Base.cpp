/*
 * Base.cpp
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#include <Base.h>

#include <ignition/math/Angle.hh>

namespace sfm {
namespace vis {

// ------------------------------------------------------------------- //

Base::Base(): frame_("map"), max_arrow_length_(1.0f), sfm_max_force_(2000.0) { }

// ------------------------------------------------------------------- //

void Base::init(const std::string &parent_frame) {
	frame_ = parent_frame;
}

// ------------------------------------------------------------------- //

void Base::setColorArrow(const float &r, const float &g, const float &b, const float &alpha) {
	setColor(&color_arrow_, r, g, b, alpha);
}

// ------------------------------------------------------------------- //

void Base::setArrowParameters(const float &length_meters, const float &sfm_max_force) {
	max_arrow_length_ = length_meters;
	sfm_max_force_ = sfm_max_force;
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Base::createArrow(const ignition::math::Vector3d &pos, const ignition::math::Vector3d &vector) const {

	visualization_msgs::Marker marker;

	// NOTE: header.stamp, ns, deprecated here
	// NOTE: marker.id is necessary for MarkerArray (otherwise only 1 marker will be drawn)
	// `ADD is something of a misnomer, it really means "create or modify"`
	// http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/Marker.html

	marker.header.frame_id = frame_;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = pos.X();
	marker.pose.position.y = pos.Y();
	marker.pose.position.z = pos.Z();

	// marker orientation is based on force vector direction
	ignition::math::Angle yaw(std::atan2(vector.Y(), vector.X()));
	yaw.Normalize();

	// convert to quaternion
	ignition::math::Quaterniond quaternion(0.0, 0.0, yaw.Radian());

	marker.pose.orientation.x = quaternion.X();
	marker.pose.orientation.y = quaternion.Y();
	marker.pose.orientation.z = quaternion.Z();
	marker.pose.orientation.w = quaternion.W();

	// scale
	// arrow's length is calculated based on max allowable force `in SFM class`
	marker.scale.x = max_arrow_length_ * vector.Length() / sfm_max_force_;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color = color_arrow_;

	return (marker);

}

// ------------------------------------------------------------------- //

void Base::setColor(std_msgs::ColorRGBA *color, const float &r, const float &g, const float &b, const float &alpha) {
	color->a = alpha;	color->r = r;	color->g = g;	color->b = b;
}

// ------------------------------------------------------------------- //

Base::~Base() { }

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace sfm */
