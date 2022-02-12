#pragma once

#include <hubero_common/time.h>
#include <hubero_common/typedefs.h>

#include <limits>

namespace hubero {

/**
 * @brief Structure that contains variables used internally for task execution
 */

class InternalMemory {
public:
	InternalMemory():
		bb_type_previous_(BasicBehaviourType::BB_UNDEFINED),
		bb_type_current_(BasicBehaviourType::BB_UNDEFINED)
	{}

	void setPoseInitial(const Pose3& pose) {
		pose_initial_ = pose;
	}

	void setTime(const Time& time) {
		time_previous_ = time_current_;
		time_current_ = time;
	}

	void setBasicBehaviour(const BasicBehaviourType& bb_type) {
		bb_type_previous_ = bb_type_current_;
		bb_type_current_ = bb_type;
	}

	void setGoal(const Pose3& goal) {
		pose_goal_ = goal;
	}

	void setPose(const Pose3& pose) {
		pose_previous_ = pose_current_;
		pose_current_ = pose;
	}

	void setGoalPoseUpdateTime(const Time& time) {
		time_goal_update_ = time;
	}

	Pose3 getPoseInitial() const {
		return pose_initial_;
	}

	Time getTimeCurrent() const {
		return time_current_;
	}

	Time getTimePrevious() const {
		return time_previous_;
	}

	Pose3 getPoseGoal() const {
		return pose_goal_;
	}

	Pose3 getPoseCurrent() const {
		return pose_current_;
	}

	Pose3 getPosePrevious() const {
		return pose_previous_;
	}

	BasicBehaviourType getBasicBehaviourCurrent() const {
		return bb_type_current_;
	}

	BasicBehaviourType getBasicBehaviourPrevious() const {
		return bb_type_previous_;
	}

	Time getGoalPoseUpdateTime() const {
		return time_goal_update_;
	}

	double getDistanceToGoal() const {
		return (pose_current_.Pos() - pose_goal_.Pos()).Length();
	}

	double getPlanarDistanceToGoal() const {
		auto pos_curr_xy = Vector3(pose_current_.Pos().X(), pose_current_.Pos().Y(), 0.0);
		auto pos_goal_xy = Vector3(pose_goal_.Pos().X(), pose_goal_.Pos().Y(), 0.0);
		return (pos_curr_xy - pos_goal_xy).Length();
	}

	double getDisplacement() const {
		return (pose_current_.Pos() - pose_previous_.Pos()).Length();
	}

	bool didBasicBehaviourChange() const {
		return bb_type_current_ != bb_type_previous_;
	}

protected:
	Pose3 pose_goal_;

	Pose3 pose_initial_;
	Pose3 pose_previous_;
	Pose3 pose_current_;

	Time time_previous_;
	Time time_current_;

	/// last time of the navigation goal update
	Time time_goal_update_;

	BasicBehaviourType bb_type_previous_;
	BasicBehaviourType bb_type_current_;
};

} // namespace hubero
