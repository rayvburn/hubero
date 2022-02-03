#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_follow_object.h>

namespace hubero {

class TaskFollowObject: public TaskEssentials<FsmFollowObject, FsmFollowObject::State, EventFsmFollowObject> {
public:
	TaskFollowObject(const double& nearby_distance = 1.0):
		TaskEssentials::TaskEssentials(TASK_FOLLOW_OBJECT),
		nearby_distance_(nearby_distance)
	{
		task_args_num_ = countArgumentsNum(&TaskFollowObject::request);
		state_bb_map_ = {
			{FsmFollowObject::State::MOVING_TO_GOAL, BasicBehaviourType::BB_MOVE_TO_GOAL},
			{FsmFollowObject::State::WAITING_FOR_MOVEMENT, BasicBehaviourType::BB_STAND}
		};
	}

	virtual bool request(const std::string& object_name) override {
		object_name_ = object_name;
		return TaskEssentials::request(object_name);
	}

	inline std::string getFollowedObjectName() const {
		return object_name_;
	}

	inline double getDistanceNearby() const {
		return nearby_distance_;
	}

protected:
	/// @brief Name of the object that was requested to follow
	std::string object_name_;

	/// @brief How close from human object must be located to consider it as 'nearby'
	double nearby_distance_;
}; // TaskFollowObject

} // namespace hubero
