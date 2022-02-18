#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/logger.h>

#include <hubero_core/internal_memory.h>

#include <hubero_interfaces/utils/task_base.h>
#include <hubero_interfaces/animation_control_base.h>
#include <hubero_interfaces/navigation_base.h>
#include <hubero_interfaces/world_geometry_base.h>

namespace hubero {

/**
 * @brief Class created to avoid code duplication - all tasks share @ref execute method
 *
 * @tparam Tfsm type of the FSM class
 * @tparam Tstate type of the enum with states
 * @tparam Tevent type of the struct with predicates
 */
template <typename Tfsm, typename Tstate, typename Tevent>
class TaskEssentials: public TaskBase {
public:
	// enum alias for easier use in orchestrating class
	using State = Tstate;

	void initialize(
		std::shared_ptr<AnimationControlBase> animation_control_ptr,
		std::shared_ptr<NavigationBase> navigation_ptr,
		std::shared_ptr<WorldGeometryBase> world_geometry_ptr,
		std::shared_ptr<InternalMemory> internal_memory_ptr
	) {
		if (
			animation_control_ptr == nullptr
			|| navigation_ptr == nullptr
			|| world_geometry_ptr == nullptr
			|| internal_memory_ptr == nullptr
		) {
			return;
		}

		animation_control_ptr_ = animation_control_ptr;
		navigation_ptr_ = navigation_ptr;
		world_geometry_ptr_ = world_geometry_ptr;
		memory_ptr_ = internal_memory_ptr;
		initialized_ = true;
	}

	void addStateTransitionHandler(const int& state_src, const int& state_dst, std::function<void()> handler) {
		fsm_.addTransitionHandler(state_src, state_dst, handler);
	}

	virtual bool execute(const Tevent& event) {
		// update internal memory before execution
		updateMemory();

		BasicBehaviourType bb_type = getBasicBehaviour();
		auto bb_handler_it = basic_behaviour_handlers_.find(bb_type);
		if (bb_handler_it == basic_behaviour_handlers_.end()) {
			HUBERO_LOG(
				"Could not find %d BB (for %d state) in BB handlers map\r\n",
				static_cast<int>(bb_type),
				static_cast<int>(getFsmState())
			);
			return false;
		}
		bb_handler_it->second();

		fsm_.process_event(event);
		return true;
	}

	inline Tstate getFsmState() const {
		return static_cast<Tstate>(fsm_.current_state());
	}

	BasicBehaviourType getBasicBehaviour() const {
		auto bb_type_it = state_bb_map_.find(getFsmState());
		if (bb_type_it == state_bb_map_.end()) {
			HUBERO_LOG("Could not find %d state in STATE -> BB map\r\n", static_cast<int>(getFsmState()));
			return BasicBehaviourType::BB_UNDEFINED;
		}
		return bb_type_it->second;
	}

	/**
	 * @brief Returns true if @ref initialize was called and valid pointers were given
	 */
	bool isInitialized() const {
		return initialized_;
	}

protected:
	/**
	 * @brief Protected ctor to make it unusable publicly
	 *
	 * @details Note to class extensions: beware of extending FSM transition handlers with shared_ptr members
	 * as they are initialized later (in @ref initialize)
	 */
	TaskEssentials(TaskType task):
		TaskBase::TaskBase(task),
		initialized_(false),
		animation_control_ptr_(nullptr),
		navigation_ptr_(nullptr),
		world_geometry_ptr_(nullptr) {}

	/**
	 * @brief Method that updates given memory buffer with task objectives stored inside a class
	 */
	virtual void updateMemory() {
		memory_ptr_->setBasicBehaviour(getBasicBehaviour());
	}

	/**
	 * @brief FSM transition handler
	 */
	void thSetupAnimation(AnimationType animation_type) {
		animation_control_ptr_->start(animation_type, memory_ptr_->getTimeCurrent());
	}

	/**
	 * @brief FSM transition handler
	 *
	 * @details Use of this method requires Internal Memory to be updated with goal pose
	 */
	void thSetupNavigation() {
		navigation_ptr_->setGoal(memory_ptr_->getPoseGoal(), navigation_ptr_->getWorldFrame());
		memory_ptr_->setGoalPoseUpdateTime(memory_ptr_->getTimeCurrent());
	}

	Tfsm fsm_;

	std::map<Tstate, BasicBehaviourType> state_bb_map_;

	bool initialized_;

	std::shared_ptr<AnimationControlBase> animation_control_ptr_;
	std::shared_ptr<NavigationBase> navigation_ptr_;
	std::shared_ptr<WorldGeometryBase> world_geometry_ptr_;
	std::shared_ptr<InternalMemory> memory_ptr_;
}; // class TaskEssential

} // namespace hubero
