/*
 * Costmap2dMultiFrame.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: rayvburn
 */

#include <actor_global_plan/Costmap2dMultiFrame.h>
#include <iostream>


// --------------------------------------------------------------------------------------

Costmap2dMultiFrame::Costmap2dMultiFrame(std::string name, tf::TransformListener& tf)
		: costmap_2d::Costmap2DROS(name, tf) { }

// --------------------------------------------------------------------------------------

void Costmap2dMultiFrame::setFrameId(const std::string &frame_id) {

	ROS_INFO("Multiframed costmap's new frame is: %s", frame_id.c_str());
	this->robot_base_frame_ = frame_id;

}

// --------------------------------------------------------------------------------------

Costmap2dMultiFrame::~Costmap2dMultiFrame() { }

// --------------------------------------------------------------------------------------
