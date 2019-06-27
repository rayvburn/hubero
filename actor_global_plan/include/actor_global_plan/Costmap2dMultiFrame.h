/*
 * Costmap2dMultiFrame.h
 *
 *  Created on: Jun 25, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_COSTMAP2DMULTIFRAME_H_
#define INCLUDE_COSTMAP2DMULTIFRAME_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

class Costmap2dMultiFrame : public costmap_2d::Costmap2DROS {

public:

	Costmap2dMultiFrame(std::string name, tf::TransformListener& tf);
	void setFrameId(const std::string &frame_id);
	virtual ~Costmap2dMultiFrame();

};

#endif /* INCLUDE_COSTMAP2DMULTIFRAME_H_ */
