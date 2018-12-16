/*
 * SocialForceModelDatabase.h
 *
 *  Created on: Dec 15, 2018
 *      Author: rayvburn
 */

#ifndef INCLUDE_SOCIALFORCEMODELUTILS_H_
#define INCLUDE_SOCIALFORCEMODELUTILS_H_

#include <string>
#include <vector>
#include <cctype> 				// isdigit()
#include <gazebo/math/Box.hh>	// Box - bounding box
#include <ignition/math.hh> 	// Pose3d

// ----------------------------------------//

namespace SocialForceModel {

const uint DB_NOT_FOUND = 65535;

enum SFMObjectType {
	SFM_STATIC_REPULSIVE = 0,
	SFM_DYNAMIC_REPULSIVE,
	SFM_STATIC_ATTRACTIVE,
	SFM_DYNAMIC_ATTRACTIVE
};

// std::tuple?
struct SFMObject {
	std::string 	base_name;		// model name from Gazebo (without an id at the end, like "cafe", not "cafe1")
	SFMObjectType 	influence;
};

std::string GetDBNameFromGazeboModelName(const std::string &_gazebo_name);
double 		GetDistanceToNearestPoint(const ignition::math::Pose3d &_pose, const gazebo::math::Box &_bb);


/*
// Takes a Gazebo's model name and converts it to the SFM DB format
std::string GetDBNameFromGazeboModelName(const std::string &_gazebo_name) {

	std::string db_name = _gazebo_name;
	while ( isdigit(db_name.back()) ) {
		db_name.pop_back();
	}
	return db_name;

}


// Function useful for adding new objects to the database
// Takes a Gazebo's model name and looks for a similar name in SFM DB convention
// Searching by name
bool IsInDatabase(const std::string &_name) {
	// will it be useful? there is a need to define object's influence on actor,
	// so all the objects must to be known before program start
	return true;
}
*/

}

#endif /* INCLUDE_SOCIALFORCEMODELUTILS_H_ */
