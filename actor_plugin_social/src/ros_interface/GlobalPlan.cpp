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

namespace actor {
namespace ros_interface {

// getWaypointHandler status indicators
const uint8_t GET_WAYPOINT_IN_PROGRESS 	= 0;
const uint8_t GET_WAYPOINT_FINISHED 	= 1;
const uint8_t GET_WAYPOINT_PATH_EMPTY 	= 2;

//#define USE_ROS_PKG

// ------------------------------------------------------------------- //

GlobalPlan::GlobalPlan(): nh_ptr_(nullptr), waypoint_curr_(0), waypoint_gap_(10), target_reached_(true) { }

// ------------------------------------------------------------------- //

GlobalPlan::GlobalPlan(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id)
		: nh_ptr_(nh_ptr), waypoint_curr_(0), waypoint_gap_(gap), target_reached_(true), frame_id_(frame_id) {

#ifdef USE_ROS_PKG
	srv_client_ = nh_ptr_->serviceClient<navfn::MakeNavPlan>("ActorGlobalPlanner");
#else
	srv_client_ = nh_ptr_->serviceClient<actor_global_plan::MakeNavPlanFrame>("ActorGlobalPlanner");
#endif

}

// ------------------------------------------------------------------- //

//void GlobalPlan::setNodeHandle(std::shared_ptr<ros::NodeHandle> nh_ptr) {
//	nh_ptr_ = nh_ptr;
//	srv_client_ = nh_ptr_->serviceClient<actor_global_plan::MakeNavPlanFrame>("ActorGlobalPlan");
//}
//
//// ------------------------------------------------------------------- //
//
//void GlobalPlan::setWaypointGap(const size_t &gap) {
//	waypoint_gap_ = gap;
//}

void GlobalPlan::initialize(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id) {

	nh_ptr_ = nh_ptr;

#ifdef USE_ROS_PKG
	srv_client_ = nh_ptr_->serviceClient<navfn::MakeNavPlan>("ActorGlobalPlanner");
#else
	srv_client_ = nh_ptr_->serviceClient<actor_global_plan::MakeNavPlanFrame>("ActorGlobalPlanner");
#endif

	waypoint_gap_ = gap;
	frame_id_ = frame_id;

}

// ------------------------------------------------------------------- //

GlobalPlan::MakePlanStatus GlobalPlan::makePlan(const ignition::math::Vector3d &start, const ignition::math::Vector3d &goal) {

	geometry_msgs::PoseStamped start_pose = converter_.convertIgnVectorToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = converter_.convertIgnVectorToPoseStamped(goal);
	return (makePlanHandler(start_pose, goal_pose));

}

// ------------------------------------------------------------------- //

GlobalPlan::MakePlanStatus GlobalPlan::makePlan(const ignition::math::Pose3d &start, const ignition::math::Pose3d &goal) {

	geometry_msgs::PoseStamped start_pose = converter_.convertIgnPoseToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = converter_.convertIgnPoseToPoseStamped(goal);
	return (makePlanHandler(start_pose, goal_pose));

}

// ------------------------------------------------------------------- //

GlobalPlan::MakePlanStatus GlobalPlan::makePlan(const ignition::math::Pose3d &start, const ignition::math::Vector3d &goal) {

	geometry_msgs::PoseStamped start_pose = converter_.convertIgnPoseToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = converter_.convertIgnVectorToPoseStamped(goal);
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

	case(GET_WAYPOINT_IN_PROGRESS):
		path_index = index;
		break;
	case(GET_WAYPOINT_FINISHED):
		path_index = path_.size() - 1;
		break;
	case(GET_WAYPOINT_PATH_EMPTY):
		return (converter_.convertPoseStampedToIgnPose(geometry_msgs::PoseStamped()));
		break;
	}

	/*
	size_t path_index = 0;

	if ( path_.size() > index ) {

		//return (path_.at(index));
		path_index = index;

	} else if ( path_.size() != 0 ) { 	//	(path_.size() <= index) <-- assured

		// one tried to get waypoint which is out of allowable index range AND path_.size() is bigger than 0
		target_reached_ = true;
		//return (path_.at(path_.size() - 1));
		path_index = path_.size() - 1;

	} else {

		// path_.size() == 0
		target_reached_ = true;
		//return (geometry_msgs::PoseStamped());
		return (convertPoseStampedToIgnPose(geometry_msgs::PoseStamped()));

	}
	*/
	return (converter_.convertPoseStampedToIgnPose(path_.at(path_index)));

}

// ------------------------------------------------------------------- //

ignition::math::Pose3d GlobalPlan::getWaypoint() {

	std::cout << "GlobalPlan::getWaypoint() - size: " << path_.size() << std::endl;

	size_t index = 0;
	uint8_t status = getWaypointHandler(waypoint_curr_);

	switch(status) {

	case(GET_WAYPOINT_IN_PROGRESS):
		index = waypoint_curr_;
		waypoint_curr_ += waypoint_gap_;
		break;
	case(GET_WAYPOINT_FINISHED):
		index = path_.size() - 1;
		break;
	case(GET_WAYPOINT_PATH_EMPTY):
		return (converter_.convertPoseStampedToIgnPose(geometry_msgs::PoseStamped()));
		break;
	}

	/*
	size_t index = 0;

	if ( path_.size() > waypoint_curr_ ) {

		index = waypoint_curr_;
		waypoint_curr_ += waypoint_gap_;

	} else if ( path_.size() != 0 ) {

		// index (waypoint_curr_) is out of allowable range
		target_reached_ = true;
		index = path_.size() - 1;

	} else {

		// path_.size() == 0
		target_reached_ = true;
		return (convertPoseStampedToIgnPose(geometry_msgs::PoseStamped()));

	}
	*/

	std::cout << "GlobalPlan::getWaypoint() - index: " << index << std::endl;
	return (converter_.convertPoseStampedToIgnPose(path_.at(index)));

}

// ------------------------------------------------------------------- //

void GlobalPlan::setPosesFrames(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal) {

	start.header.frame_id = "map";	// TODO: world
	goal.header.frame_id = "map";	// TODO: world

}

// ------------------------------------------------------------------- //

GlobalPlan::MakePlanStatus GlobalPlan::makePlanHandler(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal) {

#ifdef USE_ROS_PKG
	navfn::MakeNavPlan nav_plan;
#else
	actor_global_plan::MakeNavPlanFrame nav_plan;
//	nav_plan.request.start = start;
//	nav_plan.request.goal = goal;
//	setPosesFrames(start, goal);	// overwrite previous `frame_id` field
#endif

	nav_plan.request.start.header.frame_id = "map";
	//nav_plan.request.start.header.stamp = ros::Time::now();

	nav_plan.request.start.pose.position.x = start.pose.position.x;
	nav_plan.request.start.pose.position.y = start.pose.position.y;
	nav_plan.request.start.pose.position.z = start.pose.position.z;

	nav_plan.request.start.pose.orientation.x = start.pose.orientation.x;
	nav_plan.request.start.pose.orientation.y = start.pose.orientation.y;
	nav_plan.request.start.pose.orientation.z = start.pose.orientation.z;
	nav_plan.request.start.pose.orientation.w = start.pose.orientation.w;


	nav_plan.request.goal.header.frame_id = "map";
	//nav_plan.request.goal.header.stamp = ros::Time::now();

	nav_plan.request.goal.pose.position.x = goal.pose.position.x;
	nav_plan.request.goal.pose.position.y = goal.pose.position.y;
	nav_plan.request.goal.pose.position.z = goal.pose.position.z;

	nav_plan.request.goal.pose.orientation.x = goal.pose.orientation.x;
	nav_plan.request.goal.pose.orientation.y = goal.pose.orientation.y;
	nav_plan.request.goal.pose.orientation.z = goal.pose.orientation.z;
	nav_plan.request.goal.pose.orientation.w = goal.pose.orientation.w;

#ifndef USE_ROS_PKG
	nav_plan.request.controlled_frame = frame_id_;
#endif

	bool success = srv_client_.call(nav_plan);

	if ( success ) {

		path_.clear();
		// planner successfully found a valid path
		path_ = nav_plan.response.path;
		target_reached_ = false;
		std::cout << "\tGoal is reachable, path size: " << path_.size() << std::endl;

//		for ( size_t i = 0; i < path_.size(); i++ ) {
//			std::cout << path_.at(i).pose.position << std::endl;
//		}

		//ROS_INFO("Goal is reachable");
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
			//ROS_ERROR("Plan couldn't be calculated, error message: ");
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
		return (GET_WAYPOINT_IN_PROGRESS);

	} else if ( path_.size() != 0 ) {

		// index (waypoint_curr_) is out of allowable range
		target_reached_ = true;
		//index = path_.size() - 1;
		return (GET_WAYPOINT_FINISHED);

	} else {

		// path_.size() == 0
		target_reached_ = true;
		return (GET_WAYPOINT_PATH_EMPTY);

	}

}

// ------------------------------------------------------------------- //

GlobalPlan::~GlobalPlan() { }

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
