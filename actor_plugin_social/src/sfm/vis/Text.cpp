/*
 * Text.cpp
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#include <sfm/vis/Text.h>
#include <string>

namespace sfm {
namespace vis {

// ------------------------------------------------------------------- //

Text::Text(): text_size_(0.5) {}

// ------------------------------------------------------------------- //

void Text::setParameters(const float &text_size) {
	text_size_ = text_size;
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Text::create(const ignition::math::Vector3d &pos, const int &number) const {

	return (create(pos, std::to_string(number)));

}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Text::create(const ignition::math::Vector3d &pos, const std::string &text) const {

	visualization_msgs::Marker marker;

	// NOTE: header.stamp, ns, deprecated here
	// NOTE: marker.id is necessary for MarkerArray (otherwise only 1 marker will be drawn)
	// `ADD is something of a misnomer, it really means "create or modify"`
	// http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/Marker.html

	marker.header.frame_id = this->frame_;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = pos.X();
	marker.pose.position.y = pos.Y();
	marker.pose.position.z = pos.Z();

	// scale - height of the letter
	marker.scale.z = text_size_;

	marker.color = this->color_;

	marker.text = text;

	return (marker);

}

// ------------------------------------------------------------------- //

Text::~Text() { }

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace sfm */
