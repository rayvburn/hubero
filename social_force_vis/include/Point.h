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

// ignition libraries
#include <ignition/math/Pose3.hh>

// ROS libraries
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>


namespace sfm {
namespace vis {

/// \brief Point class extends Base's functionality
/// with a LineList management (may be developed further)
class Point : public Base {

public:

	/// \brief Default constructor
	Point();

	/// \brief Sets a color of each line created
	/// with createLineList() method
	void setColorLine(const float &r, const float &g, const float &b, const float &alpha);

	/// \brief Method which wraps createLineList()
	visualization_msgs::MarkerArray createLineListArray(const std::vector<ignition::math::Pose3d> &poses) const;

	/// \brief Method that passes position components
	/// of a poses to a createLineList() method which
	/// takes ignition::math::Vector3d
	visualization_msgs::Marker createLineList(const ignition::math::Pose3d &p1, const ignition::math::Pose3d &p2) const;

	/// \brief Function that creates a line list;
	/// each line could be created out of 2 points
	visualization_msgs::Marker createLineList(const ignition::math::Vector3d &p1, const ignition::math::Vector3d &p2) const;

	/// \brief Default destructor
	virtual ~Point();

private:

	/// \brief Stores a color a line
	std_msgs::ColorRGBA color_line_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_POINT_H_ */
