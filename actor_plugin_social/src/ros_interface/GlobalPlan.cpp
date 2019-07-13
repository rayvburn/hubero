/*
 * GlobalPlan.cpp
 *
 *  Created on: Jun 21, 2019
 *      Author: rayvburn
 */

#include "ros_interface/GlobalPlan.h"
#include <iostream>

#include <navfn/MakeNavPlan.h>
#include <actor_global_plan/MakeNavPlanFrame.h>
#include <std_srvs/Trigger.h>
#include <actor_global_plan/GetCost.h>

#include <thread> // FIXME: debugging

namespace actor {
namespace ros_interface {

// getWaypointHandler status indicators
static constexpr uint8_t GLOBAL_PLAN_GET_WAYPOINT_IN_PROGRESS 	= 0;
static constexpr uint8_t GLOBAL_PLAN_GET_WAYPOINT_FINISHED 		= 1;
static constexpr uint8_t GLOBAL_PLAN_GET_WAYPOINT_PATH_EMPTY 	= 2;

// ------------------------------------------------------------------- //

GlobalPlan::GlobalPlan(): nh_ptr_(nullptr), waypoint_curr_(0), waypoint_gap_(10), target_reached_(true) { }

// ------------------------------------------------------------------- //

GlobalPlan::GlobalPlan(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id)
		: nh_ptr_(nh_ptr), waypoint_curr_(0), waypoint_gap_(gap), target_reached_(true), frame_id_(frame_id) {

	srv_client_ = nh_ptr_->serviceClient<actor_global_plan::MakeNavPlanFrame>("ActorGlobalPlanner");
	srv_client_costmap_status_ = nh_ptr_->serviceClient<std_srvs::Trigger>("ActorGlobalPlanner/CostmapStatus");
	srv_client_get_cost_ = nh_ptr_->serviceClient<actor_global_plan::GetCost>("ActorGlobalPlanner/GetCost");

}

// ------------------------------------------------------------------- //

void GlobalPlan::initialize(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id) {

	nh_ptr_ = nh_ptr;
	srv_client_ = nh_ptr_->serviceClient<actor_global_plan::MakeNavPlanFrame>("ActorGlobalPlanner");
	srv_client_costmap_status_ = nh_ptr_->serviceClient<std_srvs::Trigger>("ActorGlobalPlanner/CostmapStatus");
	srv_client_get_cost_ = nh_ptr_->serviceClient<actor_global_plan::GetCost>("ActorGlobalPlanner/GetCost");
	waypoint_gap_ = gap;
	frame_id_ = frame_id;

}

// ------------------------------------------------------------------- //

bool GlobalPlan::isCostmapInitialized() {

	std_srvs::Trigger msg;

	// call the service server's callback
	bool success = srv_client_costmap_status_.call(msg);

	// return response flag, not success flag itself (SrvCallback always returns true)
	return (msg.response.success);

}

// ------------------------------------------------------------------- //

GlobalPlan::MakePlanStatus GlobalPlan::makePlan(const ignition::math::Vector3d &start, const ignition::math::Vector3d &goal) {

	geometry_msgs::PoseStamped start_pose = converter_.convertIgnVectorToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = converter_.convertIgnVectorToPoseStamped(goal);

	// forcing planar calculations
	start_pose.pose.position.z = 0.0;
	goal_pose.pose.position.z = 0.0;

	return (makePlanHandler(start_pose, goal_pose));

}

// ------------------------------------------------------------------- //

GlobalPlan::MakePlanStatus GlobalPlan::makePlan(const ignition::math::Pose3d &start, const ignition::math::Pose3d &goal) {

	geometry_msgs::PoseStamped start_pose = converter_.convertIgnPoseToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = converter_.convertIgnPoseToPoseStamped(goal);

	// forcing planar calculations
	start_pose.pose.position.z = 0.0;
	goal_pose.pose.position.z = 0.0;

	return (makePlanHandler(start_pose, goal_pose));

}

// ------------------------------------------------------------------- //

GlobalPlan::MakePlanStatus GlobalPlan::makePlan(const ignition::math::Pose3d &start, const ignition::math::Vector3d &goal) {

	geometry_msgs::PoseStamped start_pose = converter_.convertIgnPoseToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = converter_.convertIgnVectorToPoseStamped(goal);

	// forcing planar calculations
	start_pose.pose.position.z = 0.0;
	goal_pose.pose.position.z = 0.0;

	return (makePlanHandler(start_pose, goal_pose));

}

// ------------------------------------------------------------------- //

bool GlobalPlan::isTargetReached() const {
	return (target_reached_);
}

// ------------------------------------------------------------------- //

std::vector<geometry_msgs::PoseStamped> GlobalPlan::getPoses() const {
	return (path_);
}

// ------------------------------------------------------------------- //

nav_msgs::Path GlobalPlan::getPath() const {

	// NOTE: path is definitely related to `map` coordinate system
	nav_msgs::Path path_msg;
	path_msg.poses = path_;
	path_msg.header.frame_id = "map";
	path_msg.header.stamp = ros::Time::now();
	return (path_msg);

}

// ------------------------------------------------------------------- //

ignition::math::Pose3d GlobalPlan::getWaypoint(const size_t &index) {

	size_t path_index = 0;
	uint8_t status = getWaypointHandler(index);

	switch(status) {

	case(GLOBAL_PLAN_GET_WAYPOINT_IN_PROGRESS):
		path_index = index;
		break;
	case(GLOBAL_PLAN_GET_WAYPOINT_FINISHED):
		path_index = path_.size() - 1;
		break;
	case(GLOBAL_PLAN_GET_WAYPOINT_PATH_EMPTY):
		return (converter_.convertPoseStampedToIgnPose(geometry_msgs::PoseStamped()));
		break;
	}

	return (converter_.convertPoseStampedToIgnPose(path_.at(path_index)));

}

// ------------------------------------------------------------------- //

ignition::math::Pose3d GlobalPlan::getWaypoint() {

	size_t index = 0;
	uint8_t status = getWaypointHandler(waypoint_curr_);

	switch(status) {

	case(GLOBAL_PLAN_GET_WAYPOINT_IN_PROGRESS):
		index = waypoint_curr_;
		waypoint_curr_ += waypoint_gap_;
		break;
	case(GLOBAL_PLAN_GET_WAYPOINT_FINISHED):
		index = path_.size() - 1;
		break;
	case(GLOBAL_PLAN_GET_WAYPOINT_PATH_EMPTY):
		return (converter_.convertPoseStampedToIgnPose(geometry_msgs::PoseStamped()));
		break;
	}

	return (converter_.convertPoseStampedToIgnPose(path_.at(index)));

}

// ------------------------------------------------------------------- //

void GlobalPlan::setPosesFrames(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal) {

	start.header.frame_id = "world";
	goal.header.frame_id = "world";

}

// ------------------------------------------------------------------- //

uint8_t GlobalPlan::getCost(const double &x_world, const double &y_world) {

	actor_global_plan::GetCost cost;

	cost.request.point.x = x_world;
	cost.request.point.y = y_world;
	bool success = srv_client_get_cost_.call(cost);

	return (static_cast<uint8_t>(cost.response.cost));

}

// ------------------------------------------------------------------- //

GlobalPlan::MakePlanStatus GlobalPlan::makePlanHandler(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal) {

	actor_global_plan::MakeNavPlanFrame nav_plan;
	ros::Time time_curr = ros::Time::now();

	// fill start fields
	nav_plan.request.start.header.frame_id = "world";
	nav_plan.request.start.header.stamp = time_curr;
	nav_plan.request.start.pose = start.pose;

	// fill goal fields
	nav_plan.request.goal.header.frame_id = "world";
	nav_plan.request.goal.header.stamp = time_curr;
	nav_plan.request.goal.pose = goal.pose;

	// set a new frame for costmap
	nav_plan.request.controlled_frame = frame_id_;

	// call the service server's callback
	bool success = srv_client_.call(nav_plan);

	if ( success ) {

		path_.clear();

		// planner successfully found a valid path
		path_ = nav_plan.response.path;
		target_reached_ = false;
		waypoint_curr_ = 0;
		//std::cout << "\tGoal is reachable, path size: " << path_.size() << std::endl;
		return (MakePlanStatus::GLOBAL_PLANNER_SUCCESSFUL);

	} else {

		// planner did not find a valid path

		if ( nav_plan.response.error_message == "!GLOBAL_PLANNER_BUSY!" ) {

			// planner busy now
			std::cout << "Plan couldn't be calculated - planner is busy now" << std::endl;
			return (MakePlanStatus::GLOBAL_PLANNER_BUSY);

		} else {

			// planner failed to make a valid plan
			std::cout << "Plan couldn't be calculated, error message: " << nav_plan.response.error_message << std::endl;
			return (MakePlanStatus::GLOBAL_PLANNER_FAILED);

		}

	}

	return (MakePlanStatus::GLOBAL_PLANNER_UNKNOWN);

}

// ------------------------------------------------------------------- //

uint8_t GlobalPlan::getWaypointHandler(const size_t &index) {

	if ( path_.size() > index ) {

		//index = waypoint_curr_;
		//waypoint_curr_ += waypoint_gap_;
		return (GLOBAL_PLAN_GET_WAYPOINT_IN_PROGRESS);

	} else if ( path_.size() != 0 ) {

		// index (waypoint_curr_) is out of allowable range
		target_reached_ = true;
		//index = path_.size() - 1;
		return (GLOBAL_PLAN_GET_WAYPOINT_FINISHED);

	} else {

		// path_.size() == 0
		target_reached_ = true;
		return (GLOBAL_PLAN_GET_WAYPOINT_PATH_EMPTY);

	}

}

// ------------------------------------------------------------------- //

GlobalPlan::~GlobalPlan() { }

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
