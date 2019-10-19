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

Path::Path(const double &path_resolution): resolution_(path_resolution), updated_(false), index_(0) {}

// ------------------------------------------------------------------- //

bool Path::collect(const ignition::math::Vector3d &pos, const double &distance_to_closest_obstacle) {

	// if it is a first position then save it anyway
	// OR
	// evaluate length condition - if length between a given point
	// and the lastly saved one is bigger or equal to the `resolution`,
	// then `pos` should be saved in the path vector
	if ( ((pos - last_valid_pos_).Length() >= resolution_) || (path_.size() == 0) ) {

		last_valid_pos_ = pos;

		// indicates that a buffer is full and the oldest data should be overwritten
		if ( path_.size() <= 1000 ) {

			path_.push_back(converter_.convertIgnVectorToPoseStamped(pos, true));
			dists_.push_back(distance_to_closest_obstacle);

		} else {

			path_.at(index_) = converter_.convertIgnVectorToPoseStamped(pos, true);
			dists_.at(index_) = distance_to_closest_obstacle;
			if ( ++index_ > 1000 ) {
				index_ = 0;
			}

		}
		updated_ = true;
		return (true);

	}

	updated_ = false;
	return (false);

}

// ------------------------------------------------------------------- //

void Path::reset() {
	path_.clear();
	dists_.clear();
	index_ = 0;
	last_valid_pos_ = ignition::math::Vector3d();
	updated_ = false; // to prevent trying to get element from empty vector
}

// ------------------------------------------------------------------- //

bool Path::isUpdated() const {
	return (updated_);
}

// ------------------------------------------------------------------- //

nav_msgs::Path Path::getPath() const {

	nav_msgs::Path path_msg;
	path_msg.poses = path_;
	path_msg.header.frame_id = "map";
	path_msg.header.stamp = ros::Time::now();
	return (path_msg);

}

// ------------------------------------------------------------------- //

std::vector<double> Path::getDistances() const {
	return (dists_);
}

// ------------------------------------------------------------------- //

geometry_msgs::PoseStamped Path::getPosition() const {
	return (path_.at(path_.size() - 1));
}

// ------------------------------------------------------------------- //

double Path::getDistance() const {
	return (dists_.at(dists_.size() - 1));
}

// ------------------------------------------------------------------- //

Path::~Path() {}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
