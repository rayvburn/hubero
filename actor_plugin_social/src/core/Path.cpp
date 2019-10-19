/*
 * Path.cpp
 *
 *  Created on: Oct 19, 2019
 *      Author: rayvburn
 */

#include <core/Path.h>

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

Path::Path(const double &path_resolution): resolution_(path_resolution) {}

// ------------------------------------------------------------------- //

bool Path::collect(const ignition::math::Vector3d &pos) {

	// if it is a first position then save it anyway
	// OR
	// evaluate length condition - if length between a given point
	// and the lastly saved one is bigger or equal to the `resolution`,
	// then `pos` should be saved in the path vector
	if ( ((pos - last_valid_pos_).Length() >= resolution_) || (path_.size() == 0) ) {
		last_valid_pos_ = pos;
		path_.push_back(converter_.convertIgnVectorToPoseStamped(pos));
		return (true);
	}

	return (false);

}

// ------------------------------------------------------------------- //

void Path::collect(const double &distance_to_closest_obstacle) {
	dists_.push_back(distance_to_closest_obstacle);
}

// ------------------------------------------------------------------- //

void Path::reset() {
	path_.clear();
	dists_.clear();
	last_valid_pos_ = ignition::math::Vector3d();
}

// ------------------------------------------------------------------- //

nav_msgs::Path Path::getPath() const {
	return (path_);
}

// ------------------------------------------------------------------- //

Path::~Path() {}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
