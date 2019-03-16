/*
 * SFMVisPoint.h
 *
 *  Created on: Mar 15, 2019
 *      Author: rayvburn
 */

#ifndef SFMVISPOINT_H_
#define SFMVISPOINT_H_

#include <ignition/math.hh>	// include whole math
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace SocialForceModel {

// ------------------------------------------------------------------- //

/*
 * NOTE: Multiple actors' force visualization require SFMVisPoint to be static within the ActorPlugin
 */

class SFMVisPoint {

public:

	SFMVisPoint();
	SFMVisPoint(const std::string &_namespace_id, const std::string &_parent_frame);
	void init(const std::string &_namespace_id, const std::string &_parent_frame);

	void setMaxArrowLength(const float _marker_length);
	void setForcePoint(const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_pt, const unsigned int &_pt_id);
	visualization_msgs::MarkerArray getMarkerArray();

	virtual ~SFMVisPoint();

private:

	void clearInternalMemory();
	unsigned int getIDUpdateMapAndAction(const unsigned int &_pt_id);

	std::map<unsigned int, unsigned int> map_point_id_index;
	visualization_msgs::MarkerArray marker_array;
	int32_t action;
	float arrow_length;
	unsigned int point_last_idx;
	std::string ns;
	std::string frame;

};

} /* namespace SocialForceModel */

#endif /* SFMVISPOINT_H_ */