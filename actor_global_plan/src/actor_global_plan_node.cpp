/*
 * actor_global_plan_node.cpp
 *
 *  Created on: Jun 22, 2019
 *      Author: rayvburn
 */

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <navfn/MakeNavPlan.h>
#include <actor_global_plan/MakeNavPlanFrame.h>	// X

#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include "actor_global_plan/GlobalPlannerMultiFrame.h"
#include <costmap_2d/costmap_2d_ros.h>

// ROS Kinetic
#include <tf/transform_listener.h>

// ROS Melodic
//#include <tf2_ros/buffer.h>
//#include <tf2_ros/transform_listener.h>

// global planner ----------------------------------------------------------------------------
static GlobalPlannerMultiFrame* glob_planner_ptr_;
static std::vector<geometry_msgs::PoseStamped> path_;
static ros::ServiceServer make_plan_service_;
static bool MakePlanService(actor_global_plan::MakeNavPlanFrame::Request& req, actor_global_plan::MakeNavPlanFrame::Response& resp);

// costmap -----------------------------------------------------------------------------------
// NOTE: it was impossible to call costmap's constructor inside a separate class (outside a ROS node)
// despite providing a proper NodeHandle and TransformListener (checked, stashed among other bad commits)
static Costmap2dMultiFrame* costmap_global_ptr_;

// transform listener ------------------------------------------------------------------------
static tf::TransformListener* tf_listener_ptr_;
static std::string frame = "world";

// -------------------------------------------------------------------------------------------
// main --------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
int main(int argc, char** argv) {

	// node initialization
	ros::init(argc, argv, "actor_global_plan_node");
	ros::NodeHandle nh;

	// check if an extra (necessary) argument provided
	if ( argc < 2 ) {
		ROS_ERROR("A namespace of actors' shared NodeHandle must be provided! Service needs it. See ''actor_global_plan/launch/actor_global_plan.launch'' for details");
		return (-1);
	}
	std::string srv_ns = argv[1];

	// initialize transform listener
	tf::TransformListener tf_listener(ros::Duration(10.0));
	tf_listener_ptr_ = &tf_listener;

	// initialize global costmap
	// NOTE: costmap 2d takes tf2_ros::Buffer in ROS Melodic, in Kinetic - tf::TransformListener
	Costmap2dMultiFrame costmap_global(std::string("gcm"), tf_listener); // ("actor_global_costmap", tf_listener);
	costmap_global_ptr_ = &costmap_global;

	// initialize global planner
	GlobalPlannerMultiFrame global_planner(std::string("global_planner"), &costmap_global, frame);
	glob_planner_ptr_ = &global_planner;

	// start plan making service
	make_plan_service_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner"), MakePlanService);

	// print some info
	ROS_INFO("actor_global_plan Node started successfully");

	// process callbacks
	ros::spin();

	return 0;

}

// ----------------------------------------------------------------------------------------------------
// modified plan maker service callback ---------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
bool MakePlanService(actor_global_plan::MakeNavPlanFrame::Request& req, actor_global_plan::MakeNavPlanFrame::Response& resp) {

	// first, check whether global planner is busy at the moment
	if ( glob_planner_ptr_->isBusy() ) {
		resp.error_message = "!GLOBAL_PLANNER_BUSY!";
		resp.plan_found = false;
		return (false);
	}

	// debugging request
//	std::cout << "\tstart_pos: \n" << req.start.pose.position << "\n\tgoal_pos: \n" << req.goal.pose.position << std::endl;
//	std::cout << "\tpath_size initial: " << path_.size() << std::endl;
//	std::cout << "\tcontrolled_frame: " << req.controlled_frame << std::endl;
//	std::cout << "\n\n" << std::endl;

	// if global planner is not busy, let's try to find a path for a proper frame
	glob_planner_ptr_->setFrameId(req.controlled_frame);
	resp.plan_found = glob_planner_ptr_->makePlan(req.start, req.goal, path_);

	if ( resp.plan_found ) {

		resp.path = path_;
		ROS_INFO("Actor global planner made a plan consisting of %u points", resp.path.size());

	}

	path_.clear();
	return (resp.plan_found);

}
