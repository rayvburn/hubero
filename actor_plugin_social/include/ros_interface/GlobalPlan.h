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

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <stdint.h>
#include "ros_interface/Conversion.h"

namespace actor {
namespace ros_interface {

class GlobalPlan {

	typedef enum {
		GLOBAL_PLANNER_SUCCESSFUL = 0,
		GLOBAL_PLANNER_BUSY,
		GLOBAL_PLANNER_FAILED,
		GLOBAL_PLANNER_UNKNOWN
	} MakePlanStatus;

public:

	GlobalPlan();

	GlobalPlan(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id);

	/// \brief Loads a ros::NodeHandle pointer;
	/// A shared node is used for communication with ROS
	/// to avoid creating a separate node for each Actor
	void setNodeHandle(std::shared_ptr<ros::NodeHandle> nh_ptr); // DEPRECATED

	void setWaypointGap(const size_t &gap);	// DEPRECATED

	void initialize(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id);

	MakePlanStatus makePlan(const ignition::math::Vector3d &start, const ignition::math::Vector3d &goal);
	MakePlanStatus makePlan(const ignition::math::Pose3d &start, const ignition::math::Pose3d &goal);
	MakePlanStatus makePlan(const ignition::math::Pose3d &start, const ignition::math::Vector3d &goal);

	bool isTargetReached() const;
	std::vector<geometry_msgs::PoseStamped> getPoses() const;
	nav_msgs::Path getPath() const;

	ignition::math::Pose3d getWaypoint(const size_t &index);
	ignition::math::Pose3d getWaypoint();

	virtual ~GlobalPlan();

private:

	void setPosesFrames(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal);
	MakePlanStatus makePlanHandler(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal);
	uint8_t getWaypointHandler(const size_t &index);

	/// \brief NodeHandle's shared_ptr
	std::shared_ptr<ros::NodeHandle> nh_ptr_;
	ros::ServiceClient srv_client_;
	std::vector<geometry_msgs::PoseStamped> path_;
	actor::ros_interface::Conversion converter_;

	// helper for iteration through waypoints vector
	size_t waypoint_gap_;
	size_t waypoint_curr_;

	// flag switched to false when make plan called and then switched true when getWaypoint already went through whole path_ elements or last element already tried to be achieved
	bool target_reached_;

	std::string frame_id_;

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_GLOBALPLAN_H_ */
