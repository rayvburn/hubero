/*
 * SFMVisGrid.h
 *
 *  Created on: Mar 14, 2019
 *      Author: rayvburn
 */

#ifndef SRC_SFMVISGRID_H_
#define SRC_SFMVISGRID_H_

#include <ignition/math.hh>	// include whole math
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace SocialForceModel {

class SFMVisGrid {

public:

	SFMVisGrid();
	SFMVisGrid(const std::string &_namespace_id, const std::string &_parent_frame);
	void Init(const std::string &_namespace_id, const std::string &_parent_frame);

	void CreateGrid(const float &_x_start, const float &_x_end, const float &_y_start, const float &_y_end, const float &_resolution);
	bool IsWholeGridChecked();
	ignition::math::Vector3d GetNextGridElement();
	void ResetGridIndex();
	void SetForce(const ignition::math::Vector3d &_force);

	visualization_msgs::MarkerArray GetMarkerArray();
	virtual ~SFMVisGrid();

private:

	void ClearInternalMemory();

	std::vector<ignition::math::Vector3d> grid;
	size_t grid_index;
	visualization_msgs::MarkerArray marker_array;
	int32_t action;
	float arrow_length;
	std::string ns;
	std::string frame;

};

} /* namespace SocialForceModel */

#endif /* SRC_SFMVISGRID_H_ */
