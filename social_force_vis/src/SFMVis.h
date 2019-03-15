/*
 * SFMVis.h
 *
 *  Created on: Mar 14, 2019
 *      Author: rayvburn
 */

#ifndef SRC_SFMVIS_H_
#define SRC_SFMVIS_H_

#include <ignition/math.hh>	// include whole math
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace SocialForceModel {

class SFMVis {

public:

	SFMVis();
	void createGrid(const float &_x_start, const float &_x_end, const float &_y_start, const float &_y_end, const float &_resolution);
	bool isWholeGridChecked();
	void resetGridIndex();
	void addForce(const ignition::math::Vector3d &_force);
	visualization_msgs::MarkerArray getMarkerArray();
	ignition::math::Vector3d getNextGridElement();

	static constexpr float MAX_ARROW_LENGTH = 0.5;

	virtual ~SFMVis();

private:

	void clearInternalMemory();
	std::vector<ignition::math::Vector3d> grid;
	size_t grid_index;
	visualization_msgs::MarkerArray marker_array;
	int32_t action;

};

} /* namespace SocialForceModel */

#endif /* SRC_SFMVIS_H_ */
