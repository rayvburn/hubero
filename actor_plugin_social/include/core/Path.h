/*
 * Path.h
 *
 *  Created on: Oct 19, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_CORE_PATH_H_
#define INCLUDE_CORE_PATH_H_

#include <ignition/math/Vector3.hh>
#include <nav_msgs/Path.h>
#include <ros_interface/Conversion.h>

namespace actor {
namespace core {

class Path {

public:

	/// @brief Parametrized constructor
	/// @param path_resolution: defines minimum length between
	/// 2 consecutive path points
	Path(const double &path_resolution);

	/// @brief Tries to add a given `pos` and distance to the internal
	/// vector of positions. A new dataset is added if the vector of positions
	/// is empty or if the length between last `valid` point and the given
	/// one is big enough.
	/// @param pos: position
	/// @param distance_to_closest_obstacle: defines distance
	/// to the closest obstacle present in the world (in meters).
	/// @return True if `pos` and sitance have been added
	bool collect(const ignition::math::Vector3d &pos, const double &distance_to_closest_obstacle);

	/// @brief Clears the internal vector of positions
	void reset();

	/// @brief Returns True if internal vectors have just been updated
	bool isUpdated() const;

	/// @brief Returns path
	/// @return Vector of positions
	nav_msgs::Path getPath() const;

	/// @brief Returns vector of distances to the closest obstacle
	std::vector<double> getDistances() const;

	/// @brief Returns the most recent position
	geometry_msgs::PoseStamped getPosition() const;

	/// @brief Returns the most recent distance to an obstacle
	double getDistance() const;

	/// @brief Destructor
	virtual ~Path();

private:

	/// @brief Determines minimum length between 2 consecutive path points
	double resolution_;

	/// @brief Helper class instance for conversions from ignition Vector3
	/// to the PoseStamped
	actor::ros_interface::Conversion converter_;

	/// @brief Stores a point which is a reference in length calculation
	/// during evaluation whether to add a new point to the path.
	ignition::math::Vector3d last_valid_pos_;

	/// @brief Vector of positions (in fact poses)
	std::vector<geometry_msgs::PoseStamped> path_;

	/// @brief Vector of distances to the closest obstacle in the world
	std::vector<double> dists_;

	/// @brief Flag storing information whether internal vectors
	/// have just been updated
	bool updated_;

	/// @brief Stores an index of the vector. A new element will be added
	/// in position defined by that index
	size_t index_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_CORE_PATH_H_ */
