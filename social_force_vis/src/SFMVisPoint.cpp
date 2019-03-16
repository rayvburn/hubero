/*
 * SFMVisPoint.cpp
 *
 *  Created on: Mar 15, 2019
 *      Author: rayvburn
 */

#include "SFMVisPoint.h"

namespace SocialForceModel {

// ------------------------------------------------------------------- //

SFMVisPoint::SFMVisPoint():
		arrow_length(1.0f),
		point_last_idx(0),
		ns("social_force_model"),
		frame("map")
{
	// clear vectors
	this->clearInternalMemory();
}

// ------------------------------------------------------------------- //

SFMVisPoint::SFMVisPoint(const std::string &_namespace_id, const std::string &_parent_frame):
		arrow_length(1.0f),
		point_last_idx(0),
		ns(_namespace_id),
		frame(_parent_frame)
{
	// clear vectors
	this->clearInternalMemory();
}

// ------------------------------------------------------------------- //

void SFMVisPoint::init(const std::string &_namespace_id, const std::string &_parent_frame) {

	// clear vectors
	this->clearInternalMemory();

	this->ns = _namespace_id;
	this->frame = _parent_frame;

}

// ------------------------------------------------------------------- //

void SFMVisPoint::setMaxArrowLength(const float _marker_length) {
	this->arrow_length = _marker_length;
}

// ------------------------------------------------------------------- //

unsigned int SFMVisPoint::getIDUpdateMapAndAction(const unsigned int &_pt_id) {

	// use of map allows to pass points IDs as a `random` numbers - not need for them to be incremented by 1
	std::map<unsigned int, unsigned int>::const_iterator iter;
	iter = this->map_point_id_index.find(_pt_id);

	if ( iter != this->map_point_id_index.end() ) {

		// found at *iter
		this->action = visualization_msgs::Marker::MODIFY;
		return iter->second;

	} else {

		// not found
		this->action = visualization_msgs::Marker::ADD;
		this->map_point_id_index.insert(std::pair<unsigned int, unsigned int>(_pt_id, point_last_idx++));
		return (point_last_idx-1);

	}

}

// ------------------------------------------------------------------- //

void SFMVisPoint::setForcePoint(	const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_pt,
							const unsigned int &_pt_id) {

	unsigned int marker_arr_index = getIDUpdateMapAndAction(_pt_id);

	visualization_msgs::Marker marker;

	marker.header.frame_id = this->frame;
	marker.header.stamp = ros::Time();
	marker.ns = this->ns;
	marker.id = marker_arr_index;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = this->action;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = _pt.X();
	marker.pose.position.y = _pt.Y();
	marker.pose.position.z = _pt.Z();

	// marker orientation is based on force vector direction
	ignition::math::Angle yaw(std::atan2(_force.Y(), _force.X()));
	yaw.Normalize();

	// convert to quaternion
	ignition::math::Quaterniond quaternion(0.0, 0.0, yaw.Radian());

	marker.pose.orientation.x = quaternion.X();
	marker.pose.orientation.y = quaternion.Y();
	marker.pose.orientation.z = quaternion.Z();
	marker.pose.orientation.w = quaternion.W();

	// specify the arrow start and end points
	// marker.points[0];
	// marker.points[1];

	// scale
	// arrow's length is calculated based on max allowable force `in SFM class`
	marker.scale.x = arrow_length * _force.Length() / 2000.0; // TODO: avoid hard-coding max_force value
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color.a = 0.8; // alpha channel
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	// depending on the `action` - add new or modify present marker
	if ( this->action == visualization_msgs::Marker::ADD ) {
		this->marker_array.markers.push_back(marker);
	} else if ( this->action == visualization_msgs::Marker::MODIFY) {
		this->marker_array.markers[marker_arr_index] = marker;
	}

}

// ------------------------------------------------------------------- //

visualization_msgs::MarkerArray SFMVisPoint::getMarkerArray() {
	return (this->marker_array);
}

// ------------------------------------------------------------------- //

void SFMVisPoint::clearInternalMemory() {

	this->marker_array.markers.clear();
	this->map_point_id_index.clear();
	this->point_last_idx = 0;
	this->action = visualization_msgs::Marker::ADD;

}

// ------------------------------------------------------------------- //

SFMVisPoint::~SFMVisPoint() {
	this->clearInternalMemory();
}

} /* namespace SocialForceModel */
