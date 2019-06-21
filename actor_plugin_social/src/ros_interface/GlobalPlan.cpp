/*
 * GlobalPlan.cpp
 *
 *  Created on: Jun 21, 2019
 *      Author: rayvburn
 */

#include "ros_interface/GlobalPlan.h"
#include <iostream>
#include <thread>

namespace actor {
namespace ros_interface {

GlobalPlan::GlobalPlan(): ns_(""), waypoint_gap_(5), waypoint_curr_(0) {

	std::cout << "GlobalPlan::GlobalPlan() - default ctor" << std::endl;
	costmap_global_ptr_ = nullptr;
	glob_planner_ptr_ = nullptr;
	//tf_listener_ptr_ = nullptr;

}

// ------------------------------------------------------------------- //

// some problems with copy-constructor, related to threading
//GlobalPlan::GlobalPlan(const GlobalPlan &obj) {
//
//	nh_ptr_ = obj.nh_ptr_;
//	ns_ = obj.ns_;
//
//	glob_planner_ = obj.glob_planner_;
//	costmap_global_ = obj.costmap_global_;
//
//	tf_listener_ = obj.tf_listener_;
//
//	waypoint_gap_ = obj.waypoint_gap_;
//	waypoint_curr_ = obj.waypoint_curr_;
//
//}

// ------------------------------------------------------------------- //

GlobalPlan::GlobalPlan(std::shared_ptr<ros::NodeHandle> nh_ptr, std::shared_ptr<tf::TransformListener> tf_ptr,
		const std::string &ns, const size_t &waypoint_gap)
		: ns_(ns), waypoint_gap_(waypoint_gap), waypoint_curr_(0)
{

//	std::cout << "GlobalPlan::GlobalPlan() - 1" << std::endl;
//	std::cout << "GlobalPlan::GlobalPlan() - 2" << std::endl;
//	std::cout << "GlobalPlan::GlobalPlan() - 3" << std::endl;
//	std::cout << "GlobalPlan::GlobalPlan() - 4" << std::endl;

	std::cout << "GlobalPlan::GlobalPlan() - parametrized ctor" << std::endl;
	initializeNode(nh_ptr, tf_ptr);
	std::cout << "GlobalPlan::GlobalPlan() - parametrized ctor1" << std::endl;
	initializeCostmap();
	std::cout << "GlobalPlan::GlobalPlan() - parametrized ctor2" << std::endl;
	initializeGlobalPlanner();
	std::cout << "GlobalPlan::GlobalPlan() - parametrized ctor3" << std::endl;

}

// ------------------------------------------------------------------- //

void GlobalPlan::init(std::shared_ptr<ros::NodeHandle> nh_ptr, std::shared_ptr<tf::TransformListener> tf_ptr,
		const std::string &ns, const size_t &waypoint_gap) {

	std::cout << "GlobalPlan::GlobalPlan() - init" << std::endl;
	initializeNode(nh_ptr, tf_ptr);
	std::cout << "GlobalPlan::GlobalPlan() - init1" << std::endl;
	ns_ = ns;
	std::cout << "GlobalPlan::GlobalPlan() - init2" << std::endl;
	waypoint_gap_ = waypoint_gap;
	std::cout << "GlobalPlan::GlobalPlan() - init3" << std::endl;
	initializeCostmap();
	std::cout << "GlobalPlan::GlobalPlan() - init4" << std::endl;
	initializeGlobalPlanner();
	std::cout << "GlobalPlan::GlobalPlan() - init5" << std::endl;

}

// ------------------------------------------------------------------- //

bool GlobalPlan::makePlan(const ignition::math::Vector3d &start, const ignition::math::Vector3d &goal) {

	path_.clear();
	geometry_msgs::PoseStamped start_pose = convertIgnVectorToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = convertIgnVectorToPoseStamped(goal);
	return(glob_planner_ptr_->makePlan(start_pose, goal_pose, path_));

}

// ------------------------------------------------------------------- //

bool GlobalPlan::makePlan(const ignition::math::Pose3d &start, const ignition::math::Pose3d &goal) {

	path_.clear();
	geometry_msgs::PoseStamped start_pose = convertIgnPoseToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = convertIgnPoseToPoseStamped(goal);
	return(glob_planner_ptr_->makePlan(start_pose, goal_pose, path_));

}

// ------------------------------------------------------------------- //

bool GlobalPlan::makePlan(const ignition::math::Pose3d &start, const ignition::math::Vector3d &goal) {

	path_.clear();
	geometry_msgs::PoseStamped start_pose = convertIgnPoseToPoseStamped(start);
	geometry_msgs::PoseStamped goal_pose  = convertIgnVectorToPoseStamped(goal);
	return(glob_planner_ptr_->makePlan(start_pose, goal_pose, path_));

}

// ------------------------------------------------------------------- //

std::vector<geometry_msgs::PoseStamped> GlobalPlan::getPath() const {
	return (path_);
}

// ------------------------------------------------------------------- //

geometry_msgs::PoseStamped GlobalPlan::getWaypoint(const size_t &index) const {

	if ( path_.size() > index ) {
		return (path_.at(index));
	}
	return (geometry_msgs::PoseStamped());

}

// ------------------------------------------------------------------- //

geometry_msgs::PoseStamped GlobalPlan::getWaypoint() {

	std::cout << "GlobalPlan::getWaypoint() - size: " << path_.size() << std::endl;

	size_t index = 0;

	if ( path_.size() > waypoint_curr_ ) {

		index = waypoint_curr_;
		waypoint_curr_ += waypoint_gap_;

	} else {

		if ( path_.size() != 0 ) {
			index = path_.size() - 1;
		} else {
			return (geometry_msgs::PoseStamped());
		}

	}

	std::cout << "GlobalPlan::getWaypoint() - index: " << index << std::endl;

	return (path_.at(index));

}

// ------------------------------------------------------------------- //

geometry_msgs::PoseStamped GlobalPlan::convertIgnVectorToPoseStamped(const ignition::math::Vector3d &pos) {

	geometry_msgs::PoseStamped pose_stamped;

	pose_stamped.header.frame_id = "map"; // world?
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.pose.position.x = pos.X();
	pose_stamped.pose.position.y = pos.Y();
	pose_stamped.pose.position.z = pos.Z();
	pose_stamped.pose.orientation.x = 0.0f;
	pose_stamped.pose.orientation.y = 0.0f;
	pose_stamped.pose.orientation.z = 0.0f;
	pose_stamped.pose.orientation.w = 1.0f;

	return (pose_stamped);

}

// ------------------------------------------------------------------- //

geometry_msgs::PoseStamped GlobalPlan::convertIgnPoseToPoseStamped(const ignition::math::Pose3d &pose) {

	geometry_msgs::PoseStamped pose_stamped;

	pose_stamped.header.frame_id = "map"; // world?
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.pose.position.x = pose.Pos().X();
	pose_stamped.pose.position.y = pose.Pos().Y();
	pose_stamped.pose.position.z = pose.Pos().Z();
	pose_stamped.pose.orientation.x = pose.Rot().X();
	pose_stamped.pose.orientation.y = pose.Rot().Y();
	pose_stamped.pose.orientation.z = pose.Rot().Z();
	pose_stamped.pose.orientation.w = pose.Rot().W();

	return (pose_stamped);

}

// ------------------------------------------------------------------- //

void GlobalPlan::initializeGlobalPlanner() {
	// global planner initialization
	std::cout << "initializeGlobalPlanner() - bef costmap extraction" << std::endl;
	costmap_2d::Costmap2D* temp_costmap = costmap_global_ptr_->getCostmap();
	std::cout << "initializeGlobalPlanner() - aft costmap extraction" << std::endl;
	glob_planner_ptr_ = new global_planner::GlobalPlanner(ns_, costmap_global_ptr_->getCostmap(), "map");
	std::cout << "initializeGlobalPlanner() - aft all" << std::endl;
}

// ------------------------------------------------------------------- //

void GlobalPlan::initializeCostmap() {
	// global costmap initialization
	//costmap_global_ptr_ = new costmap_2d::Costmap2DROS(ns_, *tf_listener_ptr_);

//	ros::Time start = ros::Time::now(); // wait to build cache
//	while (ros::Duration(ros::Time::now() - start).toSec() < 1.00) {
//		std::cout << "WAITING, now: " << ros::Time::now() << " start: " << start << " diff: " << (ros::Time::now() - start) << std::endl;
//	}

	//costmap_global_ptr_ = new costmap_2d::Costmap2DROS(ns_, tf_listener_);
	//tf::TransformListener tf_list;

	//costmap_global_ptr_ = new costmap_2d::Costmap2DROS(ns_, tf_listener_);

	costmap_global_ptr_ = new costmap_2d::Costmap2DROS(ns_, *tf_listener_ptr_.get());

}

// ------------------------------------------------------------------- //

void GlobalPlan::initializeNode(std::shared_ptr<ros::NodeHandle> nh_ptr,
		std::shared_ptr<tf::TransformListener> tf_ptr) {
	nh_ptr_ = nh_ptr;
	//tf_listener_ptr_ = new tf::TransformListener(*nh_ptr.get());
	//tf_listener_ = tf::TransformListener();

	tf_listener_ptr_ = tf_ptr;
}

// ------------------------------------------------------------------- //

GlobalPlan::~GlobalPlan() {

	if ( costmap_global_ptr_ != nullptr ) {
		delete costmap_global_ptr_;
	}

	if ( glob_planner_ptr_ != nullptr ) {
		delete glob_planner_ptr_;
	}

//	if ( tf_listener_ptr_ != nullptr ) {
//		delete tf_listener_ptr_;
//	}

}

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
