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

//#define USE_ROS_PKG

// global planner ----------------------------------------------------------------------------
#ifdef USE_ROS_PKG
static global_planner::GlobalPlanner* glob_planner_ros_ptr_;
#else
static GlobalPlannerMultiFrame* glob_planner_ptr_;
#endif

static std::vector<geometry_msgs::PoseStamped> path_;

#ifdef USE_ROS_PKG
static ros::ServiceServer make_plan_service_navfn_;
#else
static ros::ServiceServer make_plan_service_;
#endif

#ifdef USE_ROS_PKG
static bool MakePlanServiceNavfn(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp);
#else
static bool MakePlanService(actor_global_plan::MakeNavPlanFrame::Request& req, actor_global_plan::MakeNavPlanFrame::Response& resp);
#endif

// costmap -----------------------------------------------------------------------------------
// NOTE: it was impossible to call its constructor inside a separate class (outside a ROS node)
#ifdef USE_ROS_PKG
static costmap_2d::Costmap2DROS* costmap_global_ros_ptr_;
#else
static Costmap2dMultiFrame* costmap_global_ptr_;
#endif

// transform listener ------------------------------------------------------------------------
static tf::TransformListener* tf_listener_ptr_;

// main --------------------------------------------------------------------------------------
int main(int argc, char** argv) {

	std::cout << "\n[actor_global_plan_node] start\n" << std::endl;

	// node initialization
	ros::init(argc, argv, "actor_global_plan_node");
	ros::NodeHandle nh; // ros::NodeHandle nh("~actor_global_plan"); //

	std::cout << "\n[actor_global_plan_node] ros inited\n" << std::endl;

	std::cout << "\n[actor_global_plan_node] args counter: " << argc << "\n" << std::endl;
	for ( size_t i = 0; i < argc; i++ ) {
		std::cout << "\n[actor_global_plan_node] arg " << i << ": " << argv[i] << "\n" << std::endl;
	}

	if ( argc < 2 ) {
		ROS_ERROR("Namespace of actors NodeHandle must be provided! Service needs it");
		return (0);
	}

	std::string srv_ns = argv[1];

	// initialize transform listener
	tf::TransformListener tf_listener(ros::Duration(10.0));
	tf_listener_ptr_ = &tf_listener;

	std::cout << "\n[actor_global_plan_node] tf listener\n" << std::endl;

	// initialize global costmap
	// NOTE: costmap 2d takes tf2_ros::Buffer in ROS Melodic, in Kinetic - tf::TransformListener
#ifdef USE_ROS_PKG
	costmap_2d::Costmap2DROS costmap_global_ros("gcm", tf_listener);
	costmap_global_ros_ptr_ = &costmap_global_ros;
#else
	Costmap2dMultiFrame costmap_global(std::string("gcm"), tf_listener); // ("actor_global_costmap", tf_listener);
	costmap_global_ptr_ = &costmap_global;
#endif

	std::cout << "\n[actor_global_plan_node] costmap\n" << std::endl;

	// initialize global planner
#ifdef USE_ROS_PKG
	global_planner::GlobalPlanner global_planner_ros("global_planner", costmap_global_ros_ptr_->getCostmap(), "map");
	glob_planner_ros_ptr_ = &global_planner_ros;
#else
	GlobalPlannerMultiFrame global_planner(std::string("global_planner"), &costmap_global, "map");
	glob_planner_ptr_ = &global_planner;
#endif

	std::cout << "\n[actor_global_plan_node] planner\n" << std::endl;

	// start plan making service
#ifdef USE_ROS_PKG
	make_plan_service_navfn_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner"), MakePlanServiceNavfn); // Navfn
	// <navfn::MakeNavPlan::Request, navfn::MakeNavPlan::Response>
#else
	make_plan_service_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner"), MakePlanService);
	// <actor_global_plan::MakeNavPlanFrame>
#endif

	std::cout << "\n[actor_global_plan_node] service srvr started\n" << std::endl;

	// process callbacks
	ros::spin();

	return 0;

}

#ifdef USE_ROS_PKG
// plan maker service callback ---------------------------------------------------------------
bool MakePlanServiceNavfn(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp) {

	std::cout << "\n\n\nMakePlanService navfn::MakeNavPlan\n\n\n" << std::endl;

	resp.plan_found = glob_planner_ros_ptr_->makePlan(req.start, req.goal, path_);

	if ( resp.plan_found ) {

		resp.path = path_;
		std::cout << "PLAN OK, LENGTH: " << resp.path.size() << std::endl;

	}

	path_.clear();
	return (resp.plan_found);

}
#else
// modified plan maker service callback ---------------------------------------------------------------
bool MakePlanService(actor_global_plan::MakeNavPlanFrame::Request& req, actor_global_plan::MakeNavPlanFrame::Response& resp) {

	std::cout << "\n\n\t\t\tMakePlanService MODIFIED!\n\n" << std::endl;

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
		std::cout << "PLAN OK, LENGTH: " << resp.path.size() << std::endl;

	}

	path_.clear();
	return (resp.plan_found);

}
#endif
