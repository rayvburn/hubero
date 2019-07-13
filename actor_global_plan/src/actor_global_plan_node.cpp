/*
 * actor_global_plan_node.cpp
 *
 *  Created on: Jun 22, 2019
 *      Author: rayvburn
 */

#include <vector>
#include <iostream>
#include <thread>	// FIXME: waiting for other nodes to be ready (especially tf_static)
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <navfn/MakeNavPlan.h>
#include <actor_global_plan/MakeNavPlanFrame.h>	// if not recognized by Eclipse: 1) recompile with catkin, 2) refresh files

#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include "actor_global_plan/GlobalPlannerMultiFrame.h"
#include <costmap_2d/costmap_2d_ros.h>

#include <std_srvs/Trigger.h> // costmap status
#include <actor_global_plan/GetCost.h> // getcost service

// ROS Kinetic
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/static_transform_broadcaster.h>

// ROS Melodic
//#include <tf2_ros/buffer.h>
//#include <tf2_ros/transform_listener.h>

// global planner ----------------------------------------------------------------------------
static GlobalPlannerMultiFrame* glob_planner_ptr_;
static std::vector<geometry_msgs::PoseStamped> path_;
static ros::ServiceServer make_plan_srv_;
static ros::ServiceServer get_cost_srv_;
static ros::ServiceServer costmap_status_srv_;
static bool MakePlanSrv(actor_global_plan::MakeNavPlanFrame::Request& req, actor_global_plan::MakeNavPlanFrame::Response& resp);
static bool CostmapStatusSrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
static bool GetCostSrv(actor_global_plan::GetCost::Request& req, actor_global_plan::GetCost::Response& resp);

// costmap -----------------------------------------------------------------------------------
// NOTE: it was impossible to call costmap's constructor inside a separate class (outside a ROS node)
// despite providing a proper NodeHandle and TransformListener (checked, stashed among other bad commits)
static Costmap2dMultiFrame* costmap_global_ptr_;
static bool costmap_ready_ = false;	// flag set true when node becomes fully initialized; ugly way but Costmap2DROS
									// has a member `initialized_` set as private, so there is no way to use it in
									// a derived class (Costmap2dMultiFrame)

// transform listener ------------------------------------------------------------------------
static tf::TransformListener* tf_listener_ptr_;
static std::string frame = "world";
static tf2_ros::TransformBroadcaster* tf_broadcaster_ptr_;

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

	// start costmap status service
	costmap_status_srv_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner/CostmapStatus"), CostmapStatusSrv);

	std::cout << "\t[global plan] transform listener" << std::endl;
	// initialize transform listener
	tf::TransformListener tf_listener(ros::Duration(10.0)); // TODO: make shorter as more stable version comes in
	tf_listener_ptr_ = &tf_listener;

	std::cout << "\t[global plan] costmap" << std::endl;
//	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	tf2_ros::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ptr_ = &tf_broadcaster;
	geometry_msgs::TransformStamped tf;
	tf.child_frame_id = "map";
	tf.header.frame_id = "world";
	tf.header.stamp = ros::Time::now();
	tf.transform.translation.x = 0.0;
	tf.transform.translation.y = 0.0;
	tf.transform.translation.z = 0.0;
	tf.transform.rotation.x = 0.0;
	tf.transform.rotation.y = 0.0;
	tf.transform.rotation.z = 0.0;
	tf.transform.rotation.w = 1.0;
	tf_broadcaster_ptr_->sendTransform(tf);
	// initialize global costmap
	// NOTE: costmap 2d takes tf2_ros::Buffer in ROS Melodic, in Kinetic - tf::TransformListener
	Costmap2dMultiFrame costmap_global(std::string("gcm"), tf_listener); // ("actor_global_costmap", tf_listener);
	costmap_global_ptr_ = &costmap_global;

	std::cout << "\t[global plan] global planner" << std::endl;
	// initialize global planner
	GlobalPlannerMultiFrame global_planner(std::string("global_planner"), &costmap_global, frame);
	glob_planner_ptr_ = &global_planner;

	std::cout << "\t[global plan] services" << std::endl;
	// start plan making and cost getter services
	make_plan_srv_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner"), MakePlanSrv);
	get_cost_srv_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner/GetCost"), GetCostSrv);

	// print some info
	ROS_INFO("actor_global_plan Node started successfully");

	// Update costmap status
	costmap_ready_ = true;

	// process callbacks
	ros::spin();

	return 0;

}

// ----------------------------------------------------------------------------------------------------
// modified plan maker service callback ---------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
static bool MakePlanSrv(actor_global_plan::MakeNavPlanFrame::Request& req, actor_global_plan::MakeNavPlanFrame::Response& resp) {

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
		ROS_INFO("Actor global planner made a plan consisting of %u points", static_cast<unsigned int>(resp.path.size()));

	}

	path_.clear();
	return (resp.plan_found);

}

// ----------------------------------------------------------------------------------------------------
// ---- costmap status service callback ---------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
static bool CostmapStatusSrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
	std::cout << "\t[global plan] CostmapStatusSrv check" << std::endl;
	resp.success = costmap_ready_;
	return (true);
}

// ----------------------------------------------------------------------------------------------------
// ---- get cost service callback ---------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
static bool GetCostSrv(actor_global_plan::GetCost::Request& req, actor_global_plan::GetCost::Response& resp) {

	resp.cost = static_cast<uint8_t>(costmap_global_ptr_->getCost(req.point.x, req.point.y));
	if ( resp.cost == 255 ) {
		resp.error_message = "[ERROR] Given position is out of bounds or no information could have been acquired";
	} else {
		resp.error_message = "OK";
	}
	return (true);

}
