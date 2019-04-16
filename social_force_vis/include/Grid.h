/*
 * Grid.h
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_GRID_H_
#define INCLUDE_GRID_H_

#include "Base.h"

// ROS libraries
#include <visualization_msgs/MarkerArray.h>


namespace sfm {
namespace vis {

class Grid : public Base {

public:

	Grid();

	void createGrid(const float &x_start, const float &x_end, const float &y_start, const float &y_end, const float &resolution);
	void addMarker(const visualization_msgs::Marker &marker);
	bool isWholeGridChecked() const;
	ignition::math::Vector3d getNextGridElement();
	void resetGridIndex();
	visualization_msgs::MarkerArray getMarkerArray() const;

	virtual ~Grid();

private:

	std::vector<ignition::math::Vector3d> grid_;
	size_t grid_index_;
	visualization_msgs::MarkerArray marker_array_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_GRID_H_ */
