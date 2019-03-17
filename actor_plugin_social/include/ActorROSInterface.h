/*
 * ActorROSInterface.h
 *
 *  Created on: Mar 17, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTORROSINTERFACE_H_
#define INCLUDE_ACTORROSINTERFACE_H_

// ------------------------------------------------------------------- //

// C++ headers
#include <vector>
#include <utility> // pair
#include <string>

// ROS headers
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>

// ignition library headers
#include <ignition/math/Pose3.hh>

// Actor-specific headers
#include "SFMVisPoint.h"
#include "SFMVisGrid.h"

// ------------------------------------------------------------------- //

namespace ActorUtils {

class ActorROSInterface {

public:

	ActorROSInterface();

	void Init(const std::string &_actor_name);

	void PublishActorTf(const ignition::math::Pose3d &_actor_pose);
	void PublishMarkerArray(const visualization_msgs::MarkerArray &_marker_array);

	void LoadParameters();
	bool AreParametersLoaded() const;

	virtual ~ActorROSInterface();

private:

	std::string actor_name;
	ros::Publisher pub_marker_array;
	tf2_ros::TransformBroadcaster tf_broadcaster;

	std::vector<std::pair<std::string, std::string> > params;

	static bool is_node_started;
	static std::unique_ptr<ros::NodeHandle> nh;

};

} /* namespace ActorUtils */

#endif /* INCLUDE_ACTORROSINTERFACE_H_ */
