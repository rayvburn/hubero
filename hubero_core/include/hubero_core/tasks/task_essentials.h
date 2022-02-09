#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/logger.h>
#include <hubero_interfaces/utils/task_base.h>
#include <hubero_core/internal_memory.h>

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

	void addStateTransitionHandler(const int& state_src, const int& state_dst, std::function<void()> handler) {
		fsm_.addTransitionHandler(state_src, state_dst, handler);
	}

	// NOTE: putting virtual here produces segfaults (most likely cause the method is not defined in derived function)
	void updateMemory(InternalMemory& memory) {
		memory.setBasicBehaviour(getBasicBehaviour());
	}

	bool execute(const Tevent& event) {
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

protected:
	/// @brief Protected ctor to make it unusable publicly
	TaskEssentials(TaskType task): TaskBase::TaskBase(task) {}

	Tfsm fsm_;

	std::map<Tstate, BasicBehaviourType> state_bb_map_;
}; // class TaskEssential

} // namespace hubero
