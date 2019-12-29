/*
 * Grid.cpp
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#include <sfm/vis/Grid.h>

namespace sfm {
namespace vis {

// ------------------------------------------------------------------- //

Grid::Grid(): grid_index_(0) {}

// ------------------------------------------------------------------- //

void Grid::createGrid(const float &x_start, const float &x_end, const float &y_start,
						const float &y_end, const float &resolution) {

	// clear vector containing grid and markers in case of grid resize
	grid_.clear();
	marker_array_.markers.clear();

	float x_start_temp 	= x_start;
	float x_end_temp 	= x_end;
	float y_start_temp 	= y_start;
	float y_end_temp 	= y_end;

	// prepare data - calculate bounds' values from low to high
	if ( x_end < x_start ) {
		x_start_temp = x_end;
		x_end_temp 	= x_start;
	}

	if ( y_end < y_start ) {
		y_start_temp = y_end;
		y_end_temp	= y_start;
	}

	// according to resolution, create grid (measurement) points within selected bounds
	for ( float x = x_start_temp; x <= x_end_temp; x += std::fabs(resolution) ) {
		for ( float y = y_start_temp; y <= y_end_temp; y += std::fabs(resolution) ) {
			grid_.push_back( ignition::math::Vector3d(static_cast<double>(x), static_cast<double>(y), 0.0) );
		}
	}

	// resize the marker array
	marker_array_.markers.resize( grid_.size() );

}

// ------------------------------------------------------------------- //

void Grid::addMarker(const visualization_msgs::Marker &marker) {

	/* create a temporary variable to change an ID;
	 * leaving ID constant here will allow drawing only 1 marker
	 * even when a MarkerArray is big */

	visualization_msgs::Marker marker_temp = std::move(marker);
	marker_temp.id = grid_index_ - 1;
	marker_array_.markers.at( grid_index_ - 1 ) = marker_temp;

}

// ------------------------------------------------------------------- //

bool Grid::isWholeGridChecked() const {
	if ( grid_index_ >= grid_.size() ) {
		return (true);
	}
	return (false);
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Grid::getNextGridElement() {
	grid_index_++;
	return ( grid_.at(grid_index_ - 1) );
}

// ------------------------------------------------------------------- //

void Grid::resetGridIndex() {
	grid_index_ = 0;
}

// ------------------------------------------------------------------- //

visualization_msgs::MarkerArray Grid::getMarkerArray() const {
	return ( marker_array_ );
}

// ------------------------------------------------------------------- //

Grid::~Grid() { }

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace sfm */
