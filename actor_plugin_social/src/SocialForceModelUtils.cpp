/*
 * SocialForceModelUtils.cpp
 *
 *  Created on: Dec 15, 2018
 *      Author: rayvburn
 */

#include <SocialForceModelUtils.h>

namespace SocialForceModel {

// Takes a Gazebo's model name and converts it to the SFM DB format
std::string GetDBNameFromGazeboModelName(const std::string &_gazebo_name) {

	std::string db_name = _gazebo_name;
	while ( isdigit(db_name.back()) ) {
		db_name.pop_back();
	}
	return db_name;

}

}

// Takes the actor's current pose and calculates the distance to the closest point of an object's bounding box
double GetDistanceToNearestPoint(const ignition::math::Pose3d &_pose, const gazebo::math::Box &_bb) {

	return 0.0F;
}


