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

	GlobalPlannerMultiFrame(std::string name, Costmap2dMultiFrame* costmap, std::string frame_id);

	bool isBusy() const;
	bool setFrameId(const std::string &frame_id);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
    // redefinition needed to save `busy` flag which determines whether global planner
    // instance could be used by another actor

	virtual ~GlobalPlannerMultiFrame();

protected:

	// indicates whether other frame is currently assigned to costmap and using it
	bool busy_;

	bool cm_initialized_;

	Costmap2dMultiFrame* costmap_mf_ptr_;

};

#endif /* INCLUDE_GLOBALPLANNERMULTIFRAME_H_ */
