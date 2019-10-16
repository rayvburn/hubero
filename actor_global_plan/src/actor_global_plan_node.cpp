/*
 * actor_global_plan_node.cpp
 *
 *  Created on: Jun 22, 2019
 *      Author: rayvburn
 */

#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <navfn/MakeNavPlan.h>
#include <actor_global_plan/MakeNavPlanFrame.h>	// if not recognized by Eclipse: 1) recompile with catkin, 2) refresh files

#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include "actor_global_plan/GlobalPlannerMultiFrame.h"
#include <costmap_2d/costmap_2d_ros.h>

#include <std_srvs/Trigger.h> 				// Costmap status
#include <actor_global_plan/GetCost.h> 		// GetCost service

// ROS Kinetic
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS Melodic
//#include <tf2_ros/buffer.h>
//#include <tf2_ros/transform_listener.h>

// global planner ----------------------------------------------------------------------------
static std::string* node_name_;
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

// NOTE: Firstly the footprint size was changed but this parameter (`footprint` / `robot_radius`)
// has only graphical interpretation (plan still strikes too close to the obstacle even if footprint
// was set very big.
// So as a hack the `inflation_radius` is modified so area close to obstacles is marked as `no-go`.
// This is marked as FIXME, but must be checked at least.
// NOTE: with a single actor in the world everything is the same.
static void SetInflationRadius(ros::NodeHandle &nh, const std::string &srv_ns);

// transform listener ------------------------------------------------------------------------
static tf::TransformListener* tf_listener_ptr_;
static std::string frame = "world";
static tf2_ros::TransformBroadcaster* tf_broadcaster_ptr_;
static void SendTfBlank();

// -------------------------------------------------------------------------------------------
// main --------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
int main(int argc, char** argv) {

	// node initialization
	ros::init(argc, argv, "actor_global_plan_node");
	ros::NodeHandle nh;


	// check if an extra (necessary) argument(s) provided
	if ( argc < 2 ) {
		ROS_ERROR("The namespace of actors' shared NodeHandle must be provided! Service needs it. See ''actor_global_plan/launch/actor_global_plan.launch'' for details");
		return (-1);
	}
	if ( argc < 3 ) {
		ROS_ERROR("The name of the node must be provided! See ''actor_global_plan/launch/actor_global_plan.launch'' for details");
		return (-1);
	}

	std::string srv_ns = argv[1];
	std::string node_name = argv[2];
	node_name_ = &node_name;


	// start costmap status service
	costmap_status_srv_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner/CostmapStatus"), CostmapStatusSrv);


	// initialize transform listener
	tf::TransformListener tf_listener(ros::Duration(5.0)); // TODO: make shorter as more stable version comes in
	tf_listener_ptr_ = &tf_listener;


	// initialize broadcaster and send a blank TF (NOTE: it may be deleted if static_tf_publisher works OK, see below)
	tf2_ros::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ptr_ = &tf_broadcaster;
	SendTfBlank(); // this is invoked just in case if `tf_static_publisher` did not start up before costmap initialization process

	// initialize global costmap
	// NOTE: costmap 2d takes tf2_ros::Buffer in ROS Melodic, in Kinetic - tf::TransformListener
	SetInflationRadius(nh, srv_ns); // NOTE: it's safer to call this before costmap ctor
	Costmap2dMultiFrame costmap_global(std::string("gcm"), *tf_listener_ptr_); // ("actor_global_costmap", tf_listener);
	costmap_global_ptr_ = &costmap_global;


	// initialize global planner
	GlobalPlannerMultiFrame global_planner(std::string("global_planner"), &costmap_global, frame);
	glob_planner_ptr_ = &global_planner;



	// start plan making and cost getter services
	make_plan_srv_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner"), MakePlanSrv);
	get_cost_srv_ = nh.advertiseService(std::string(srv_ns + "/ActorGlobalPlanner/GetCost"), GetCostSrv);


	// print some info
	ROS_INFO("actor_global_plan Node started successfully");

	// Update costmap status
	costmap_ready_ = true;

	// process all callbacks in an instant
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

	// set status in response message
	resp.success = costmap_ready_;
	return (true);

}

// ----------------------------------------------------------------------------------------------------
// ---- get cost service callback ---------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
static bool GetCostSrv(actor_global_plan::GetCost::Request& req, actor_global_plan::GetCost::Response& resp) {

	resp.cost = static_cast<int16_t>(costmap_global_ptr_->getCost(req.point.x, req.point.y));
	if ( resp.cost == 255 ) {
		resp.error_message = "[ERROR] Given position is out of bounds or no information could have been acquired";
	} else {
		resp.error_message = "OK";
	}
	return (true);

}
// ----------------------------------------------------------------------------------------------------
// ---- blank transform sender ------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
static void SendTfBlank() {

	geometry_msgs::TransformStamped tf;

	// NOTE: by default costmap initialized with base frame set to `map`
	// (see parameters (.yaml) in `config` folder
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

}
// ----------------------------------------------------------------------------------------------------
// ---- costmap's `inflation_radius` setter -------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------
/**
 * @brief Costmap2d's 'inflation_radius' parameter setter. This parameter determines
 * how much gap (at least) is preserved from the closest obstacle.
 *
 * @details This operation is performed in here instead of actor::ros_interface::GlobalPlan class
 * to avoid checking if `inflation_radius` was set in each iteration (Gazebo plugin must wait until
 * global_plan_node gets fully initialized, so tolerance setup can't be done in the Actor
 * constructor - only during the first "valid" OnUpdate action). "Valid" means here that ROS node
 * was already initialized.
 *
 *
 * @param nh: NodeHandle instance (in global namespace)
 * @param srv_ns: namespace services are located in (parameters as well)
 */
static void SetInflationRadius(ros::NodeHandle &nh, const std::string &srv_ns) {

	// stores bounding type id (default value is invalid)
	int bounding_type = 100;

	if ( !nh.getParam(srv_ns + "/actor/inflation/bounding_type", bounding_type) ) {
		ROS_ERROR("'bounding_type' parameter could not be found, 'default_tolerance' planner parameter will not be set");
		return;
	}

	// stores tolerance parameter (default value is invalid)
	float inflation = -1.0f;

	// helper variables (to not allocate them inside switch statement)
	double size   = -1.0;
	double size_x = -1.0;
	double size_y = -1.0;

	// to store parameters linked into vectors
	XmlRpc::XmlRpcValue list;
	XmlRpc::XmlRpcValue sublist;

	/* See "gazebo_ros_people_sim/actor_plugin_social/include/core/Enums.h"
	 * for description. If some changes in "Enums.h" were applied the 'switch'
	 * statement's cases must be adjusted accordingly. */
	switch ( bounding_type ) {

	/* ACTOR_BOUNDING_BOX */
	case(0):

		if ( !nh.getParam(srv_ns + "/actor/inflation/box_size", list) ) {
			ROS_ERROR("BoundingBox'es size could not be found");
			return;
		}

		sublist = list[0]; // only 1 element expected
		size_x = static_cast<double>( sublist["x_half"] );
		size_y = static_cast<double>( sublist["y_half"] );
		size = std::max(size_x, size_y);
		inflation = 2.15f * size * (std::sqrt(2));
		break;

	/* ACTOR_BOUNDING_CIRCLE */
	case(1):

		if ( !nh.getParam(srv_ns + "/actor/inflation/circle_radius", size) ) {
			ROS_ERROR("BoundingCircle's size could not be found");
			return;
		}
		inflation = 2.15f * size;
		break;

	/* ACTOR_BOUNDING_ELLIPSE */
	case(2):

		if ( !nh.getParam(srv_ns + "/actor/inflation/ellipse", list) ) {
			ROS_ERROR("BoundingEllipse's size could not be found");
			return;
		}

		sublist = list[0]; // only 1 element expected
		size_x = static_cast<double>( sublist["semi_major"] );
		size_y = static_cast<double>( sublist["semi_minor"] );
		size = std::min(size_x, size_y);	// broader workspace
		// size = std::max(size_x, size_y); // problematic for costmap - really narrow workspace
		inflation = 2.15f * size;
		break;

	/* ACTOR_NO_BOUNDING */
	case(3):

		inflation = 0.01f;
		break;

	/* UNKNOWN id */
	default:

		return;
		break;

	}

	// update parameter if `getParam()` was successful
	nh.setParam(*node_name_ + "/gcm/inflation_layer/inflation_radius", inflation);
	nh.setParam(*node_name_ + "/gcm/robot_radius", inflation);

}
