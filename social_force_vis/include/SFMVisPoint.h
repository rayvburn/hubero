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
	void Init(const std::string &_namespace_id, const std::string &_parent_frame);

	void SetColor(const float &_red, const float &_green, const float &_blue, const float &_alpha);
	void SetMaxArrowLength(const float _marker_length);
	void SetForcePoint(const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_pt, const unsigned int &_pt_id);
	void SetPointArrow(const ignition::math::Pose3d &_pt, const unsigned int &_pt_id);
	void SetPointsLines(const ignition::math::Pose3d &_pt1, const ignition::math::Pose3d &_pt2, const unsigned int &_pt_id);

	visualization_msgs::MarkerArray GetMarkerArray() const;
	visualization_msgs::Marker GetBBMarkerConversion(const ignition::math::Box &_bb) const;

	virtual ~SFMVisPoint();

private:

	void ClearInternalMemory();
	unsigned int GetIDUpdateMapAndAction(const unsigned int &_pt_id);

	std::map<unsigned int, unsigned int> map_point_id_index;
	visualization_msgs::MarkerArray marker_array;
	int32_t action;
	float arrow_length;
	unsigned int point_last_idx;
	std::string ns;
	std::string frame;

	typedef struct {
		float alpha = 1.0f;
		float red   = 1.0f;
		float green = 1.0f;
		float blue  = 1.0f;
	} MarkerColor;

	MarkerColor color;


};

} /* namespace SocialForceModel */

#endif /* SFMVISPOINT_H_ */
