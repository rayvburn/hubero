/*
 * GlobalPlannerMultiFrame.cpp
 *
 *  Created on: Jun 25, 2019
 *      Author: rayvburn
 */

#include <actor_global_plan/GlobalPlannerMultiFrame.h>

GlobalPlannerMultiFrame::GlobalPlannerMultiFrame(std::string name, Costmap2dMultiFrame* costmap, std::string frame_id)
		: cm_initialized_(false), global_planner::GlobalPlanner(name, costmap->getCostmap(), frame_id), busy_(false)
{

	this->costmap_ = costmap->getCostmap();
	costmap_mf_ptr_ = costmap;
	cm_initialized_ = true;

}

bool GlobalPlannerMultiFrame::isBusy() const {
	return (busy_ && cm_initialized_);
}

void GlobalPlannerMultiFrame::setFrameId(const std::string &frame_id) {

//	// safe way
//	if ( !busy_ ) {
//		costmap_mf_ptr_->setFrameId(frame_id);
//		return (true);
//	}
//	return (false);

	// brute-force way - assumed that planner is not busy which has to be checked before setFrameId() call
	costmap_mf_ptr_->setFrameId(frame_id);

}

bool GlobalPlannerMultiFrame::makePlan(const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

	// set `busy` flag
	busy_ = true;

	// save status of the plan
	bool success = global_planner::GlobalPlanner::makePlan(start, goal, plan);

	// clear `busy` flag
	busy_ = false;

	// return status
	return (success);

}

GlobalPlannerMultiFrame::~GlobalPlannerMultiFrame() {}

