/*
 * Base.h
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_BASE_H_
#define INCLUDE_BASE_H_

// C++ STL libraries
#include <string>

// ignition libraries
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// ROS libraries
#include <visualization_msgs/Marker.h>

namespace sfm {
namespace vis {


class Base {

public:

	Base();

	void init(const std::string &parent_frame);
	void setColorArrow(const float &r, const float &g, const float &b, const float &alpha);
	void setArrowParameters(const float &length_meters, const float &sfm_max_force);

	visualization_msgs::Marker createArrow(const ignition::math::Vector3d &pos, const ignition::math::Vector3d &vector) const;

	virtual ~Base();

protected:

	void setColor(std_msgs::ColorRGBA *color, const float &r, const float &g, const float &b, const float &alpha);

	std::string frame_;
	float max_arrow_length_;
	double sfm_max_force_;

	std_msgs::ColorRGBA color_arrow_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_BASE_H_ */
