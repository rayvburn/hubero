/*
 * Point.h
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_POINT_H_
#define INCLUDE_POINT_H_

#include "Base.h"

// C++ STL libraries
#include <vector>

// ROS libraries
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>


namespace sfm {
namespace vis {

class Point : public Base {

public:

	Point();

	void setColorLine(const float &r, const float &g, const float &b, const float &alpha);

	visualization_msgs::MarkerArray createLineListArray(const std::vector<ignition::math::Pose3d> &poses) const;
	visualization_msgs::Marker createLineList(const ignition::math::Pose3d &p1, const ignition::math::Pose3d &p2) const;
	visualization_msgs::Marker createLineList(const ignition::math::Vector3d &p1, const ignition::math::Vector3d &p2) const;

	virtual ~Point();

private:

	std_msgs::ColorRGBA color_line_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_POINT_H_ */
