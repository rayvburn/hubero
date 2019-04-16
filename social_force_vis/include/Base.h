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

// ROS libraries
#include <visualization_msgs/Marker.h>


namespace sfm {
namespace vis {

/// \brief Base class for sfm::vis:: objects;
/// provides some generic methods
class Base {

public:

	/// \brief Default constructor
	Base();

	/// \brief Sets parent frame (TF), default is "map"
	void init(const std::string &parent_frame);

	/// \brief Sets a color of each arrow created
	/// with createArrow() method
	void setColorArrow(const float &r, const float &g, const float &b, const float &alpha);

	/// \brief Sets arrow parameters
	/// \param[in] Max length of an arrow expressed in meters
	/// \param[in] SFM max force is a max allowable force
	/// which SFM algorithm could return; used to scale
	/// arrow's length
	void setArrowParameters(const float &length_meters, const float &sfm_max_force);

	/// \brief Method that creates a simple arrow
	/// of a previously set color
	/// \param[in] Foothold of an arrow
	/// \param[in] Force vector which is used to
	/// determine a marker's orientation and length
	visualization_msgs::Marker createArrow(const ignition::math::Vector3d &pos, const ignition::math::Vector3d &vector) const;

	/// \brief Default destructor
	virtual ~Base();

protected:

	/// \brief Helper function to set a color
	/// of a ColorRGBA instances in derived classes
	void setColor(std_msgs::ColorRGBA *color, const float &r, const float &g, const float &b, const float &alpha);

	/// \brief Stores parent frame's name
	std::string frame_;

	/// \brief Max length of an arrow (in meters]
	float max_arrow_length_;

	/// \brief Max allowable force which SFM
	/// algorithm could return
	double sfm_max_force_;

	/// \brief Stores a color an arrow
	std_msgs::ColorRGBA color_arrow_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_BASE_H_ */
