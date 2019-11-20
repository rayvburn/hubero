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
#include <tf2_ros/buffer.h>

/**
 * Costmap2dMultiFrame is a costmap_2d's extension which provides plan calculation
 * for multiple frames (not only for single robot) without the need for running multiple costmaps
 */
class Costmap2dMultiFrame : public costmap_2d::Costmap2DROS {

public:

	/**
	 * @brief Costmap2dMultiFrame - parametrized constructor
	 * @param name - namespace name, which the costmap will create a private NodeHandle in
	 * and will look for parameters somewhere in {CALLING_NODE_NAMESPACE}/{COSTMAP_NAMESPACE}
	 * @param tf - TransformListener instance
	 */

	// ROS Kinetic
	// Costmap2dMultiFrame(std::string name, tf::TransformListener& tf);
	// ROS Melodic
	Costmap2dMultiFrame(std::string name, tf2_ros::Buffer& tf_buffer);

	/**
	 * @brief Sets a Costmap2D's protected variable to a given frame_id which allows to calculate plans
	 * for multiple frames without running multiple costmaps
	 * @param frame_id - frame associated with a actor for which a plan needs to be calculated
	 */
	void setFrameId(const std::string &frame_id);

	/**
	 * @brief Converts world coordinates to map cell and checks it's cost
	 * @param x_world
	 * @param y_world
	 * @return
	 */
	unsigned char getCost(const double &x_world, const double &y_world);

	/**
	 * @brief ~Costmap2dMultiFrame() - default destructor
	 */
	virtual ~Costmap2dMultiFrame();

};

#endif /* INCLUDE_COSTMAP2DMULTIFRAME_H_ */
