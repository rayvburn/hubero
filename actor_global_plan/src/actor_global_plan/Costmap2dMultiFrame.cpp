/*
 * Costmap2dMultiFrame.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: rayvburn
 */

#include <actor_global_plan/Costmap2dMultiFrame.h>
#include <iostream>


// --------------------------------------------------------------------------------------

// ROS Kinetic
// Costmap2dMultiFrame::Costmap2dMultiFrame(std::string name, tf::TransformListener& tf)
// 		: costmap_2d::Costmap2DROS(name, tf) { }
// --------------------------------------------------------------------------------------

// ROS Melodic
Costmap2dMultiFrame::Costmap2dMultiFrame(std::string name, tf2_ros::Buffer& tf_buffer)
		: costmap_2d::Costmap2DROS(name, tf_buffer) { }

// --------------------------------------------------------------------------------------

void Costmap2dMultiFrame::setFrameId(const std::string &frame_id) {

	ROS_INFO("Multiframed costmap's new frame is: %s", frame_id.c_str());
	this->robot_base_frame_ = frame_id;

}

// --------------------------------------------------------------------------------------

unsigned char Costmap2dMultiFrame::getCost(const double &x_world, const double &y_world) {

	unsigned int x_map, y_map = 0;
	bool within_bounds = false;

	within_bounds = this->layered_costmap_->getCostmap()->worldToMap(x_world, y_world, x_map, y_map);
	if ( !within_bounds ) {
		return (255); // hard coded
	}

	return (this->layered_costmap_->getCostmap()->getCost(x_map, y_map));

}


// --------------------------------------------------------------------------------------

Costmap2dMultiFrame::~Costmap2dMultiFrame() { }

// --------------------------------------------------------------------------------------
