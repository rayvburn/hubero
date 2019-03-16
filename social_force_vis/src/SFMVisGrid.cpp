/*
 * SFMVisGrid.cpp
 *
 *  Created on: Mar 14, 2019
 *      Author: rayvburn
 */

#include "SFMVisGrid.h"
#include <tgmath.h>		// fabs()

namespace SocialForceModel {

// ------------------------------------------------------------------- //

SFMVisGrid::SFMVisGrid():
		grid_index(0),
		arrow_length(0.0f),
		ns("social_force_model"),
		frame("map")
{
	// clear vectors
	this->clearInternalMemory();
}

// ------------------------------------------------------------------- //

SFMVisGrid::SFMVisGrid(const std::string &_namespace_id, const std::string &_parent_frame):
		grid_index(0),
		arrow_length(0.0f),
		ns(_namespace_id),
		frame(_parent_frame)
{
	this->clearInternalMemory();
}

// ------------------------------------------------------------------- //

/*
 * init must be invoked each time when frame changes
 */
void SFMVisGrid::init(const std::string &_namespace_id, const std::string &_parent_frame) {

	// clear vectors
	this->clearInternalMemory();

	this->ns = _namespace_id;
	this->frame = _parent_frame;

}

// ------------------------------------------------------------------- //

/*
 * Grid allows to simulate actor's position in each grid cell (representing different world positions)
 * and visualize the force that could influence him
 */
void SFMVisGrid::createGrid(const float &_x_start, const float &_x_end, const float &_y_start,
						const float &_y_end, const float &_resolution) {

	// clear vectors containing grid and markers
	this->clearInternalMemory();

	float x_start 	= _x_start;
	float x_end 	= _x_end;
	float y_start 	= _y_start;
	float y_end 	= _y_end;

	// prepare data - calculate bounds values from low to high
	if ( _x_end < _x_start ) {
		x_start = _x_end;
		x_end 	= _x_start;
	}

	if ( _y_end < _y_start ) {
		y_start = _y_end;
		y_end	= _y_start;
	}

	// resolution becomes the arrow length
	this->arrow_length = _resolution;

	// according to resolution, create grid (measurement) points within selected bounds
	for ( float x = x_start; x < x_end; x += std::fabs(_resolution) ) {
		for ( float y = y_start; y < y_end; y += std::fabs(_resolution) ) {
			this->grid.push_back(ignition::math::Vector3d(	static_cast<double>(x),
															static_cast<double>(y),
															static_cast<double>(0.0)) );
		}
	}

	// resize the marker array
	this->marker_array.markers.resize(this->grid.size());

}

// ------------------------------------------------------------------- //

void SFMVisGrid::setForce(const ignition::math::Vector3d &_force) {

	visualization_msgs::Marker marker;

	marker.header.frame_id = this->frame;
	marker.header.stamp = ros::Time();
	marker.ns = this->ns;
	marker.id = (this->grid_index - 1);
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = this->action;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = this->grid[grid_index - 1].X();
	marker.pose.position.y = this->grid[grid_index - 1].Y();
	marker.pose.position.z = this->grid[grid_index - 1].Z();

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
	// length is calculated based on max allowable force `in SFM class`
	marker.scale.x = arrow_length * _force.Length() / 2000.0; // TODO: avoid hard-coding max_force value
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color.a = 0.7; // alpha channel
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	this->marker_array.markers[grid_index - 1] = marker;

}

// ------------------------------------------------------------------- //

visualization_msgs::MarkerArray SFMVisGrid::getMarkerArray() {
	return (this->marker_array);
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SFMVisGrid::getNextGridElement() {

	this->grid_index++;
	return (this->grid[grid_index - 1]);

}

// ------------------------------------------------------------------- //

bool SFMVisGrid::isWholeGridChecked() {

	if ( this->grid_index >= this->grid.size() ) {

		// after reaching of the last index next arrows will be modified (not added)
		this->action = visualization_msgs::Marker::MODIFY;
		return (true);

	}

	return (false);

}

// ------------------------------------------------------------------- //

void SFMVisGrid::resetGridIndex() {

	this->grid_index = 0;

	// after reset of the last index next arrows will be modified (not added)
	this->action = visualization_msgs::Marker::MODIFY;

}

// ------------------------------------------------------------------- //

SFMVisGrid::~SFMVisGrid() {

	this->clearInternalMemory();

}

// ------------------------------------------------------------------- //

void SFMVisGrid::clearInternalMemory() {

	this->grid.clear();
	this->marker_array.markers.clear();
	this->grid_index = 0;
	this->arrow_length = 0.0f;
	this->action = visualization_msgs::Marker::ADD;	// after clearing next arrows will be added

}

// ------------------------------------------------------------------- //

} /* namespace SocialForceModel */
