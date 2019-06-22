/*
 * actor_global_plan_node.cpp
 *
 *  Created on: Jun 22, 2019
 *      Author: rayvburn
 */

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <navfn/MakeNavPlan.h>

#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <costmap_2d/costmap_2d_ros.h>

// ROS Kinetic
#include <tf/transform_listener.h>

// ROS Melodic
//#include <tf2_ros/buffer.h>
//#include <tf2_ros/transform_listener.h>

// global planner ----------------------------------------------------------------------------
static global_planner::GlobalPlanner* glob_planner_ptr_;
static std::vector<geometry_msgs::PoseStamped> path_;
static ros::ServiceServer make_plan_service_;
bool MakePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp);

// costmap -----------------------------------------------------------------------------------
// NOTE: it was impossible to call its constructor inside a separate class (outside a ROS node)
static costmap_2d::Costmap2DROS* costmap_global_ptr_;

// transform listener ------------------------------------------------------------------------
static tf::TransformListener* tf_listener_ptr_;

// main --------------------------------------------------------------------------------------
int main(int argc, char** argv) {

	// node initialization
	ros::init(argc, argv, "actor_global_plan_node");
	ros::NodeHandle nh;

	// initialize transform listener
	tf::TransformListener tf_listener(ros::Duration(10.0));
	tf_listener_ptr_ = &tf_listener;

	// initialize global costmap
	// NOTE: costmap 2d takes tf2_ros::Buffer in ROS Melodic, in Kinetic - tf::TransformListener
	costmap_2d::Costmap2DROS costmap_global("actor_global_costmap", tf_listener);
	costmap_global_ptr_ = &costmap_global;

	// initialize global planner
	global_planner::GlobalPlanner global_planner("actor_global_planner", costmap_global_ptr_->getCostmap(), "map");
	glob_planner_ptr_ = &global_planner;

	// start plan making service
	make_plan_service_ = nh.advertiseService("ActorGlobalPlan", MakePlanService);

	// process callbacks
	ros::spin();

	return 0;

}

// plan maker service callback ---------------------------------------------------------------
bool MakePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp) {

	resp.plan_found = glob_planner_ptr_->makePlan(req.start, req.goal, path_);

	if ( resp.plan_found ) {

		resp.path = path_;
		std::cout << "PLAN OK, LENGTH: " << resp.path.size() << std::endl;

	}

	path_.clear();
	return (resp.plan_found);

}
