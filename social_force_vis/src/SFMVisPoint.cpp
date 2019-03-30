/*
 * SFMVisPoint.cpp
 *
 *  Created on: Mar 15, 2019
 *      Author: rayvburn
 */

#include "SFMVisPoint.h"
#include <geometry_msgs/Point.h> // points + line list

namespace SocialForceModel {

// ------------------------------------------------------------------- //

SFMVisPoint::SFMVisPoint():
		arrow_length(1.0f),
		point_last_idx(0),
		ns("social_force_model"),
		frame("map")
{
	// clear vectors
	this->ClearInternalMemory();
}

// ------------------------------------------------------------------- //

SFMVisPoint::SFMVisPoint(const std::string &_namespace_id, const std::string &_parent_frame):
		arrow_length(1.0f),
		point_last_idx(0),
		ns(_namespace_id),
		frame(_parent_frame)
{
	// clear vectors
	this->ClearInternalMemory();
}

// ------------------------------------------------------------------- //

void SFMVisPoint::Init(const std::string &_namespace_id, const std::string &_parent_frame) {

	// clear vectors
	this->ClearInternalMemory();

	this->ns = _namespace_id;
	this->frame = _parent_frame;

}

// ------------------------------------------------------------------- //

void SFMVisPoint::SetColor(const float &_red, const float &_green, const float &_blue, const float &_alpha) {

	this->color.alpha = _alpha;
	this->color.red   = _red;
	this->color.green = _green;
	this->color.blue  = _blue;

}

// ------------------------------------------------------------------- //

void SFMVisPoint::SetMaxArrowLength(const float _marker_length) {
	this->arrow_length = _marker_length;
}

// ------------------------------------------------------------------- //

unsigned int SFMVisPoint::GetIDUpdateMapAndAction(const unsigned int &_pt_id) {

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

void SFMVisPoint::SetForcePoint(	const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_pt,
							const unsigned int &_pt_id) {

	unsigned int marker_arr_index = GetIDUpdateMapAndAction(_pt_id);

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

	marker.color.a = this->color.alpha; // 0.8; // alpha channel
	marker.color.r = this->color.red;   // 0.0;
	marker.color.g = this->color.green; // 1.0;
	marker.color.b = this->color.blue;  // 0.0;

	// depending on the `action` - add new or modify present marker
	if ( this->action == visualization_msgs::Marker::ADD ) {
		this->marker_array.markers.push_back(marker);
	} else if ( this->action == visualization_msgs::Marker::MODIFY) {
		this->marker_array.markers[marker_arr_index] = marker;
	}

}

// ------------------------------------------------------------------- //

void SFMVisPoint::SetPointArrow(const ignition::math::Pose3d &_pt, const unsigned int &_pt_id) {

	unsigned int marker_arr_index = GetIDUpdateMapAndAction(_pt_id);

	visualization_msgs::Marker marker;

	marker.header.frame_id = this->frame;
	marker.header.stamp = ros::Time();
	marker.ns = this->ns;
	marker.id = marker_arr_index;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = this->action;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = _pt.Pos().X();
	marker.pose.position.y = _pt.Pos().Y();
	marker.pose.position.z = _pt.Pos().Z();

	marker.pose.orientation.x = _pt.Rot().X();
	marker.pose.orientation.y = _pt.Rot().Y();
	marker.pose.orientation.z = _pt.Rot().Z();
	marker.pose.orientation.w = _pt.Rot().W();

	// scale
	// arrow's length is calculated based on max allowable force `in SFM class`
	marker.scale.x = arrow_length; // TODO: avoid hard-coding max_force value
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color.a = this->color.alpha; // 0.8; // alpha channel
	marker.color.r = this->color.red;   // 0.0;
	marker.color.g = this->color.green; // 1.0;
	marker.color.b = this->color.blue;  // 0.0;

	// depending on the `action` - add new or modify present marker
	if ( this->action == visualization_msgs::Marker::ADD ) {
		this->marker_array.markers.push_back(marker);
	} else if ( this->action == visualization_msgs::Marker::MODIFY) {
		this->marker_array.markers[marker_arr_index] = marker;
	}

}

// ------------------------------------------------------------------- //

void SFMVisPoint::SetPointsLines(const ignition::math::Pose3d &_pt1, const ignition::math::Pose3d &_pt2,
								 const unsigned int &_pt_id)
{

	unsigned int marker_arr_index = GetIDUpdateMapAndAction(_pt_id);

	visualization_msgs::Marker marker;

	marker.header.frame_id = this->frame;
	marker.header.stamp = ros::Time();
	marker.ns = this->ns;
	marker.id = marker_arr_index;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = this->action;

	marker.pose.orientation.w = 1.0;

	// line width
	marker.scale.x = 0.05;

	marker.color.a = this->color.alpha; // 0.8; // alpha channel
	marker.color.r = this->color.red;   // 0.0;
	marker.color.g = this->color.green; // 1.0;
	marker.color.b = this->color.blue;  // 0.0;

	geometry_msgs::Point point;

	point.x = _pt1.Pos().X();
	point.y = _pt1.Pos().Y();
	point.z = 0.0f;

	marker.points.push_back(point);

	point.x = _pt2.Pos().X();
	point.y = _pt2.Pos().Y();
	point.z = 0.0f;

	marker.points.push_back(point);

	// depending on the `action` - add new or modify present marker
	if ( this->action == visualization_msgs::Marker::ADD ) {
		this->marker_array.markers.push_back(marker);
	} else if ( this->action == visualization_msgs::Marker::MODIFY) {
		this->marker_array.markers[marker_arr_index] = marker;
	}

}

// ------------------------------------------------------------------- //

visualization_msgs::MarkerArray SFMVisPoint::GetMarkerArray() const {
	return (this->marker_array);
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker SFMVisPoint::GetBBMarkerConversion(const ignition::math::Box &_bb) const {

	visualization_msgs::Marker marker;

	marker.header.frame_id = this->frame;
	marker.header.stamp = ros::Time();
	marker.ns = this->ns;
	// marker.id = marker_arr_index;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::MODIFY;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = _bb.Center().X();
	marker.pose.position.y = _bb.Center().Y();
	marker.pose.position.z = _bb.Center().Z();

	marker.pose.orientation.x = 0.00;
	marker.pose.orientation.y = 0.00;
	marker.pose.orientation.z = 0.00;
	marker.pose.orientation.w = 1.00;

	// scale
	marker.scale.x = std::fabs(_bb.Max().X() - _bb.Min().X());
	marker.scale.y = std::fabs(_bb.Max().Y() - _bb.Min().Y());
	marker.scale.z = std::fabs(_bb.Max().Z() - _bb.Min().Z());

	marker.color.a = 0.3; // alpha channel
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;

	return (marker);

}

// ------------------------------------------------------------------- //

void SFMVisPoint::ClearInternalMemory() {

	this->marker_array.markers.clear();
	this->map_point_id_index.clear();
	this->point_last_idx = 0;
	this->action = visualization_msgs::Marker::ADD;

}

// ------------------------------------------------------------------- //

SFMVisPoint::~SFMVisPoint() {
	this->ClearInternalMemory();
}

} /* namespace SocialForceModel */
