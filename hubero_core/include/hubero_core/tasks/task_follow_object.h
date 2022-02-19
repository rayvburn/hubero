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

		fsm_.addTransitionHandler(
			TaskFollowObject::State::FINISHED,
			TaskFollowObject::State::MOVING_TO_GOAL,
			std::bind(&TaskFollowObject::thSetupNavigation, this)
		);
		fsm_.addTransitionHandler(
			TaskFollowObject::State::FINISHED,
			TaskFollowObject::State::MOVING_TO_GOAL,
			std::bind(&TaskFollowObject::thSetupAnimation, this, ANIMATION_WALK)
		);
		fsm_.addTransitionHandler(
			TaskFollowObject::State::WAITING_FOR_MOVEMENT,
			TaskFollowObject::State::MOVING_TO_GOAL,
			std::bind(&TaskFollowObject::thSetupAnimation, this, ANIMATION_WALK)
		);
		fsm_.addTransitionHandler(
			TaskFollowObject::State::MOVING_TO_GOAL,
			TaskFollowObject::State::FINISHED,
			std::bind(&TaskFollowObject::thSetupAnimation, this, ANIMATION_STAND)
		);
		fsm_.addTransitionHandler(
			TaskFollowObject::State::MOVING_TO_GOAL,
			TaskFollowObject::State::WAITING_FOR_MOVEMENT,
			std::bind(&TaskFollowObject::thSetupAnimation, this, ANIMATION_STAND)
		);
	}

	virtual bool request(const std::string& object_name) override {
		object_name_ = object_name;
		return TaskEssentials::request(object_name);
	}

	/// Prepare FSM event and call @ref execute
	void execute() {
		EventFsmFollowObject event(*this, navigation_ptr_->getFeedback());
		event.setObjectNearby(memory_ptr_->getPlanarDistanceToGoal() <= navigation_ptr_->getGoalTolerance());
		TaskEssentials::execute(event);
	}

	inline std::string getFollowedObjectName() const {
		return object_name_;
	}

protected:
	virtual void updateMemory() override {
		auto object = world_geometry_ptr_->getModel(object_name_);
		memory_ptr_->setGoal(object.getPose());
		TaskEssentials::updateMemory();
	}

	/// @brief Name of the object that was requested to follow
	std::string object_name_;
}; // TaskFollowObject

} // namespace hubero
