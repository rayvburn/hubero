#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_core/tasks/task_essentials.h>
#include <hubero_core/fsm/fsm_follow_object.h>

namespace hubero {

class TaskFollowObject: public TaskEssentials<FsmFollowObject, FsmFollowObject::State, EventFsmFollowObject> {
public:
	TaskFollowObject(): TaskEssentials::TaskEssentials(TASK_FOLLOW_OBJECT) {
		task_args_num_ = countArgumentsNum(&TaskFollowObject::request);
		state_bb_map_ = {
			{FsmFollowObject::State::MOVING_TO_GOAL, BasicBehaviourType::BB_FOLLOW_OBJECT},
			{FsmFollowObject::State::WAITING_FOR_MOVEMENT, BasicBehaviourType::BB_STAND},
			{FsmFollowObject::State::FINISHED, BasicBehaviourType::BB_STAND}
		};
	}

	void updateMemory(InternalMemory& memory, const std::shared_ptr<const WorldGeometryBase> world_geometry_ptr) {
		auto object = world_geometry_ptr->getModel(object_name_);
		memory.setGoal(object.getPose());
		TaskEssentials::updateMemory(memory, world_geometry_ptr);
	}

	virtual bool request(const std::string& object_name) override {
		object_name_ = object_name;
		return TaskEssentials::request(object_name);
	}

	inline std::string getFollowedObjectName() const {
		return object_name_;
	}

protected:
	/// @brief Name of the object that was requested to follow
	std::string object_name_;
}; // TaskFollowObject

} // namespace hubero
