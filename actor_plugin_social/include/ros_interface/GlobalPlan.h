/*
 * GlobalPlan.h
 *
 *  Created on: Jun 21, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ROS_INTERFACE_GLOBALPLAN_H_
#define INCLUDE_ROS_INTERFACE_GLOBALPLAN_H_


#include <ros/ros.h>
#include <memory>
#include <chrono>	// time getter

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <stdint.h>
#include "ros_interface/Conversion.h"

namespace actor {
namespace ros_interface {

/**
 * @brief Calls a makePlan service hosted by GlobalPlanner and stores poses vector
 * created by plan maker
 */
class GlobalPlan {

public:

	/**
	 * @brief Indicates the status of makePlan method
	 */
	typedef enum {
		GLOBAL_PLANNER_SUCCESSFUL = 0,//!< GLOBAL_PLANNER_SUCCESSFUL
		GLOBAL_PLANNER_BUSY,          //!< GLOBAL_PLANNER_BUSY
		GLOBAL_PLANNER_FAILED,        //!< GLOBAL_PLANNER_FAILED
		GLOBAL_PLANNER_UNKNOWN        //!< GLOBAL_PLANNER_UNKNOWN
	} MakePlanStatus;

public:

	/**
	 * @brief Default constructor
	 */
	GlobalPlan();

	/**
	 * @brief Parametrized constructor
	 * @param nh_ptr - a shared node used for communication with ROS to avoid creating a separate one for each actor
	 * @param gap - how many poses to skip when reading a consecutive waypoint from poses vector
	 * @param frame_id - name of the frame for which plan needs to be found (usually actor's name)
	 */
	GlobalPlan(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id);

	/**
	 * @brief Initializer method (same as parametrized constructor)
	 * @param nh_ptr - a shared node used for communication with ROS to avoid creating a separate one for each actor
	 * @param gap - how many poses to skip when reading a consecutive waypoint from poses vector
	 * @param frame_id - name of the frame for which plan needs to be found (usually actor's name)
	 */
	void initialize(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id);

	/**
	 * @brief Calls service to check whether global costmap was initialized
	 * @return True when costmap is already running, false otherwise
	 */
	bool isCostmapInitialized();

	/**
	 * @brief Runs a conversion from ignition::math::Vector3d to geometry_msgs::PoseStamped,
	 * calls GlobalPlanner service and saves poses vector when plan was found
	 * @param start - starting position
	 * @param goal - goal position
	 * @return - status based on GlobalPlanner response (see MakePlanStatus enum)
	 */
	MakePlanStatus makePlan(const ignition::math::Vector3d &start, const ignition::math::Vector3d &goal);

	/**
	 * @brief Runs a conversion from ignition::math::Pose3d to geometry_msgs::PoseStamped,
	 * calls GlobalPlanner service and saves poses vector when plan was found
	 * @param start - starting pose
	 * @param goal - goal pose
	 * @return - status based on GlobalPlanner response (see MakePlanStatus enum)
	 */
	MakePlanStatus makePlan(const ignition::math::Pose3d &start, const ignition::math::Pose3d &goal);

	/**
	 * @brief Runs a conversions from ignition::math::Pose3d and ignition::math::Vector3d
	 * to geometry_msgs::PoseStamped, calls GlobalPlanner service and saves poses vector when plan was found
	 * @param start - starting pose
	 * @param goal - goal pose
	 * @return - status based on GlobalPlanner response (see MakePlanStatus enum)
	 */
	MakePlanStatus makePlan(const ignition::math::Pose3d &start, const ignition::math::Vector3d &goal);

	/**
	 * Indicates whether a full poses vector has already been checked via getWaypoint() calls
	 * If first call after finding a valid plan reaches maximum index of poses vector then
	 * `target_reached_` flag is set `true`
	 * @return True if whole poses vector was already been checked
	 */
	bool isTargetReached() const;

	/**
	 * @brief Returns poses vector
	 * @return Vector of poses which calculated plan consists of
	 */
	std::vector<geometry_msgs::PoseStamped> getPoses() const;

	/**
	 * @brief Returns a nav_msgs::Path message
	 * @return nav_msgs::Path message with a new header (filled during call) and current vector of poses
	 */
	nav_msgs::Path getPath() const;

	/**
	 * Returns a single element of poses vector
	 * Not recommended
	 * @param index - index of element from poses vector
	 * @return pose
	 */
	ignition::math::Pose3d getWaypoint(const size_t &index);

	/**
	 * Returns next element of poses vector, index of the element is increased by `gap` value given in ctor
	 * @return pose
	 */
	ignition::math::Pose3d getWaypoint();

	/**
	 * @brief Asks a ROS service server what cost does have a certain point in the map/
	 * @param x_world - x-coordinate of a point to check
	 * @param y_world - y-coordinate of a point to check
	 * @return Cost value of a point with given coordinates. Returns a value in 0-255 range.
	 * @note Some often used cost values can be found at:
	 * https://github.com/ros-planning/navigation/blob/kinetic-devel/costmap_2d/include/costmap_2d/cost_values.h
	 * @note It must not be `const`.
	 */
	uint8_t getCost(const double &x_world, const double &y_world);

	/**
	 * @brief Destructor
	 */
	virtual ~GlobalPlan();

private:

	/**
	 * @brief Runs actions after conversion of `start` and `goal` to geometry_msgs::PoseStamped
	 * @param start
	 * @param goal
	 * @return status
	 */
	MakePlanStatus makePlanHandler(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal);

	/**
	 * @brief Runs actions common between both `getWaypoint()` methods
	 * @param index - current index (a given one or `waypoint_curr_`)
	 * @return status (see source file for details)
	 */
	uint8_t getWaypointHandler(const size_t &index);

	/**
	 * @brief NodeHandle's shared_ptr
	 */
	std::shared_ptr<ros::NodeHandle> nh_ptr_;

	/**
	 * @brief ROS Service client which connects to ActorGlobalPlanner server
	 */
	ros::ServiceClient srv_client_;

	/**
	 * @brief ROS Service client which connects to ActorGlobalPlanner/CostmapStatus server
	 */
	ros::ServiceClient srv_client_costmap_status_;

	/**
	 * @brief ROS Service client which connects to ActorGlobalPlanner/CostmapStatus server
	 */
	ros::ServiceClient srv_client_get_cost_;

	/**
	 * @brief Vector of poses
	 */
	std::vector<geometry_msgs::PoseStamped> path_;

	/**
	 * @brief Helper class providing some conversions to/from geometry_msgs::PoseStamped
	 */
	actor::ros_interface::Conversion converter_;

	/**
	 * @brief Helper variable for iteration through waypoints vector; waypoint index is increased by its value
	 */
	size_t waypoint_gap_;

	/**
	 * @brief Helper variable for iteration through waypoints vector; stores an index of the next pose
	 */
	size_t waypoint_curr_;

	/**
	 * @brief Flag set to `false` during makePlan call and then switched `true` when getWaypoint already
	 * went through all `path_` elements or last element already tried to be achieved (explicitly via index)
	 */
	bool target_reached_;

	/**
	 * @brief Actor's name indicating it's frame - must be a valid name recognized
	 * by tf::TransformListener (or tf2_ros::Buffer)
	 */
	std::string frame_id_;


};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_GLOBALPLAN_H_ */
