/*
 * Costmap2dMultiFrame.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: rayvburn
 */

#include <actor_global_plan/Costmap2dMultiFrame.h>
#include <iostream>


Costmap2dMultiFrame::Costmap2dMultiFrame(std::string name, tf::TransformListener& tf)
		: costmap_2d::Costmap2DROS(name, tf)
{

	std::cout << "\n\n\n\nCostmap2dMultiFrame::Costmap2dMultiFrame(2 args)\n\n\n\n" << std::endl;

}


void Costmap2dMultiFrame::setFrameId(const std::string &frame_id) {
	std::cout << "\n\n\n\nCostmap2dMultiFrame::setFrameId()\n\n\n\n" << std::endl;
	this->robot_base_frame_ = frame_id;
}


Costmap2dMultiFrame::~Costmap2dMultiFrame() { }

