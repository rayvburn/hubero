/*
 * Border.cpp
 *
 *  Created on: Dec 10, 2019
 *      Author: rayvburn
 */

#include <actor/inflation/Border.h>

namespace actor {
namespace inflation {

// default constructor
Border::Border(): type_(BORDER_NONE) {};

// copy constructor
Border::Border(const Border &obj) {
	type_ = obj.type_;
}

BorderType Border::getType() const {
	return (type_);
}

// virtual method
void Border::updatePose(const ignition::math::Pose3d &new_pose) {
}

// virtual method
bool Border::doesContain(const ignition::math::Vector3d &pt) const {
	return (false);
}

// virtual method
std::tuple<bool, ignition::math::Vector3d> Border::doesIntersect(const ignition::math::Line3d &line) const {
	return (std::make_tuple(false, ignition::math::Vector3d()));
}

// virtual method
std::tuple<bool, ignition::math::Vector3d> Border::doesIntersect(const ignition::math::Vector3d &pt_dest) const {
	return (std::make_tuple(false, ignition::math::Vector3d()));
}

// virtual method
ignition::math::Vector3d Border::getCenter() const {
	return (ignition::math::Vector3d());
}

// virtual method
visualization_msgs::Marker Border::getMarkerConversion() const {
	return (visualization_msgs::Marker());
}

// virtual method
ignition::math::Box Border::getBox() const {
	return (ignition::math::Box());
}

// virtual method
Border::~Border() {};


}; /* namespace inflation */
}; /* namespace actor     */


