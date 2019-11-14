/*
 * Base.h
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_VIS_MARKERBASE_H_
#define INCLUDE_SFM_VIS_MARKERBASE_H_

// C++ STL libraries
#include <string>

// ignition libraries
#include <ignition/math/Vector3.hh>

// ROS libraries
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace sfm {
namespace vis {

/// \brief Base class for sfm::vis:: objects;
/// provides some generic methods
class MarkerBase {

public:

	/// \brief Default constructor
	MarkerBase();

	/// \brief Sets parent frame (TF), default is "map"
	void init(const std::string &parent_frame);

	/// \brief Sets a color of each arrow created
	/// with createArrow() method
	void setColor(const float &r, const float &g, const float &b, const float &alpha);

	/// \brief Default destructor
	virtual ~MarkerBase();

protected:

	/// \brief Helper function to set a color
	/// of a ColorRGBA instances in derived classes
	void setColor(std_msgs::ColorRGBA *color, const float &r, const float &g, const float &b, const float &alpha);

	/// \brief Stores parent frame's name
	std::string frame_;

	/// \brief Stores a color an arrow
	std_msgs::ColorRGBA color_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_SFM_VIS_MARKERBASE_H_ */
