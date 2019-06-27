/*
 * GlobalPlannerMultiFrame.h
 *
 *  Created on: Jun 25, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_GLOBALPLANNERMULTIFRAME_H_
#define INCLUDE_GLOBALPLANNERMULTIFRAME_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include "actor_global_plan/Costmap2dMultiFrame.h"

/**
 * Global planner node runs a service for plan making
 * which should be available for all actors in the world;
 * based on that requirement the costmap's base frame (stored in costmap's
 * protected variable `robot_base_frame_` should be allowed to be changed
 * based on plan `requestor`
 *
 */
class GlobalPlannerMultiFrame : public global_planner::GlobalPlanner {

public:

	/**
	 * @brief GlobalPlannerMultiFrame - constructor, takes the same parameters as GlobalPlanner's
	 * one and GlobalPlanner's `initialize` method
	 * @param name - name of the planner defining its namespace
	 * @param costmap - pointer to the multiframed costmap which is further converted to typical costmap_2d
	 * @param frame_id - frame of the costmap
	 */
	GlobalPlannerMultiFrame(std::string name, Costmap2dMultiFrame* costmap, std::string frame_id);

	/**
	 * @brief Checks whether planner is currently calculating plan for another object (actor)
	 * @return False when able to make a new plan
	 */
	bool isBusy() const;

	/**
	 * @brief Sets a new frame for which costmap will produce costs in reference to "map" frame
	 * @param frame_id - frame of the object
	 */
	void setFrameId(const std::string &frame_id);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * This method overrides GlobalPlanner's makePlan method because `busy_` flag needs to be set (and then cleared) inside;
     * a `busy_` flag is used to indicate whether GlobalPlanner could be used at the moment
     * @param start The start pose
     * @param goal The goal pose
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief Destructor
     */
	virtual ~GlobalPlannerMultiFrame();

protected:

	/**
	 * @brief Flag which indicates whether other frame is currently assigned to costmap and using it
	 */
	bool busy_;

	/**
	 * @brief Flag which indicates whether costmap has already been initialized
	 */
	bool cm_initialized_;

	/**
	 * @brief Pointer to a multi-framed costmap's instance
	 * it needs to be stored separately to provide ability to call modified costmap's `setFrameId()` method
	 * which was not previously defined (even as virtual);
	 * this is an ugly way to avoid adding a single virtual method `setFrameId` to source code of whole
	 * ROS navigation stack metapackage (only `costmap2d_ros` package needs to be modified)
	 */
	Costmap2dMultiFrame* costmap_mf_ptr_;

};

#endif /* INCLUDE_GLOBALPLANNERMULTIFRAME_H_ */
