/*
 * GlobalPlan.h
 *
 *  Created on: Jun 21, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ROS_INTERFACE_GLOBALPLAN_H_
#define INCLUDE_ROS_INTERFACE_GLOBALPLAN_H_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <navfn/MakeNavPlan.h>
#include <ros/ros.h>

#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <string>
#include <costmap_2d/costmap_2d_ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf/transform_listener.h>

#include "Node.h" //

namespace actor {
namespace ros_interface {

// NOTE: costmap 2d takes tf2_ros::Buffer in ROS Melodic
// in Kinetic - tf::TransformListener
class GlobalPlan {

public:

	GlobalPlan();
	//GlobalPlan(const GlobalPlan &obj);
	GlobalPlan(std::shared_ptr<ros::NodeHandle> nh_ptr, std::shared_ptr<tf::TransformListener> tf_ptr, const std::string &ns, const size_t &waypoint_gap);

	void init(std::shared_ptr<ros::NodeHandle> nh_ptr, std::shared_ptr<tf::TransformListener> tf_ptr, const std::string &ns, const size_t &waypoint_gap);

	bool makePlan(const ignition::math::Vector3d &start, const ignition::math::Vector3d &goal);
	bool makePlan(const ignition::math::Pose3d &start, const ignition::math::Pose3d &goal);
	bool makePlan(const ignition::math::Pose3d &start, const ignition::math::Vector3d &goal);

	std::vector<geometry_msgs::PoseStamped> getPath() const;
	geometry_msgs::PoseStamped getWaypoint(const size_t &index) const;
	geometry_msgs::PoseStamped getWaypoint();

	virtual ~GlobalPlan();

private:

	// helper methods
	geometry_msgs::PoseStamped convertIgnVectorToPoseStamped(const ignition::math::Vector3d &pos);
	geometry_msgs::PoseStamped convertIgnPoseToPoseStamped(const ignition::math::Pose3d &pose);

	void initializeGlobalPlanner();
	void initializeCostmap();
	void initializeNode(std::shared_ptr<ros::NodeHandle> nh_ptr, std::shared_ptr<tf::TransformListener> tf_ptr);

	// node
	/// \brief NodeHandle's shared_ptr required by transform listener
	std::shared_ptr<ros::NodeHandle> nh_ptr_;
	std::string ns_; // params?

	// global planner
	global_planner::GlobalPlanner* glob_planner_ptr_;
	std::vector<geometry_msgs::PoseStamped> path_;

	// costmap
	costmap_2d::Costmap2DROS* costmap_global_ptr_;

	// transforms
	//tf::TransformListener* tf_listener_ptr_;
	//tf::TransformListener tf_listener_{ros::Duration(10)};
	std::shared_ptr<tf::TransformListener> tf_listener_ptr_;

	// helper for iteration through waypoints vector
	size_t waypoint_gap_;
	size_t waypoint_curr_;

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_GLOBALPLAN_H_ */
