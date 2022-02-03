#pragma once

#include <hubero_core/events/event_fsm_super.h>
#include <hubero_core/fsm/fsm_essentials.h>

// fsmlite
#include <fsm.h>

#include <algorithm>
#include <string>

namespace hubero {

/**
 * @brief Super FSM - orchestrates highest layer of the Hierarchical Finite State Machine (HFSM)
 */
class FsmSuper: public fsmlite::fsm<FsmSuper>, public FsmEssentials {
public:
	/**
	 * @brief Enum with super-states definitions
	 */
	enum State {
		STAND = 0,
		MOVE_TO_GOAL,
		MOVE_AROUND,
		FOLLOW_OBJECT,
		LIE_DOWN,
		SIT_DOWN,
		RUN,
		TALK,
		TELEOP
	};

	FsmSuper(State state_init = State::STAND);

protected:
	/**
	 * @defgroup terminalconditions Terminal conditions
	 * @{
	 */
	/**
	 * @brief Generic calculation of terminal condition of @ref task
	 * @param event: full set of predicates
	 * @param task: task of which terminal condition is being evaluated
	 */
	static bool helperTerminalCondition(const EventFsmSuper& event, const TaskPredicates& task) {
		return task.finished || task.aborted || FsmSuper::anotherTaskRequested(event, task);
	}

	/**
	 * @brief Evaluates request predicate in 'self' (caller task) and checks how many requests there are in @ref event.
	 * @details Aim is to find whether there is another task requested currently.
	 */
	static bool anotherTaskRequested(const EventFsmSuper& event, const TaskPredicates& task_self);

	/** @} */ // end of terminalconditions group

	/**
	 * @defgroup transitionconditions Transition Conditions
	 * @details Transition from task to idle ('stand') is always allowed, FSM is just waiting for the task termination.
	 * On the other hand, to switch from idle ('stand') to certain task execution, the task must be requested first.
	 * @{
	 */
	static bool helperTransCondStand2Task(const TaskPredicates& predicates) {
		return predicates.requested;
	}

	static bool helperTransCondTask2Stand(const TaskPredicates& /* predicates */) {
		return true;
	}

	/** @} */ // end of transitionconditions group

	/**
	 * @defgroup guards FSM transition guards
	 * @details Guards combine terminal and transition conditions for a certain states transition
	 * @{
	 */
	inline bool guardStand2MoveToGoal(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.stand)
			&& FsmSuper::helperTransCondStand2Task(event.move_to_goal);
	}

	inline bool guardMoveToGoal2Stand(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.move_to_goal)
			&& FsmSuper::helperTransCondTask2Stand(event.stand);
	}

	inline bool guardStand2MoveAround(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.stand)
			&& FsmSuper::helperTransCondStand2Task(event.move_around);
	}

	inline bool guardMoveAround2Stand(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.move_around)
			&& FsmSuper::helperTransCondTask2Stand(event.stand);
	}

	inline bool guardStand2FollowObject(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.stand)
			&& FsmSuper::helperTransCondStand2Task(event.follow_object);
	}

	inline bool guardFollowObject2Stand(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.follow_object)
			&& FsmSuper::helperTransCondTask2Stand(event.stand);
	}

	inline bool guardStand2LieDown(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.stand)
			&& FsmSuper::helperTransCondStand2Task(event.lie_down);
	}

	inline bool guardLieDown2Stand(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.lie_down)
			&& FsmSuper::helperTransCondTask2Stand(event.stand);
	}

	inline bool guardStand2SitDown(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.stand)
			&& FsmSuper::helperTransCondStand2Task(event.sit_down);
	}

	inline bool guardSitDown2Stand(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.sit_down)
			&& FsmSuper::helperTransCondTask2Stand(event.stand);
	}

	inline bool guardStand2Run(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.stand)
			&& FsmSuper::helperTransCondStand2Task(event.run);
	}

	inline bool guardRun2Stand(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.run)
			&& FsmSuper::helperTransCondTask2Stand(event.stand);
	}

	inline bool guardStand2Talk(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.stand)
			&& FsmSuper::helperTransCondStand2Task(event.talk);
	}

	inline bool guardTalk2Stand(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.talk)
			&& FsmSuper::helperTransCondTask2Stand(event.stand);
	}

	inline bool guardStand2Teleop(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.stand)
			&& FsmSuper::helperTransCondStand2Task(event.teleop);
	}

	inline bool guardTeleop2Stand(const EventFsmSuper& event) const {
		return FsmSuper::helperTerminalCondition(event, event.teleop)
			&& FsmSuper::helperTransCondTask2Stand(event.stand);
	}

	/** @} */ // end of guards group

	/**
	 * @defgroup Transition handlers
	 * @{
	 */
	void transHandlerStand2MoveToGoal(const EventFsmSuper& event);
	void transHandlerStand2MoveAround(const EventFsmSuper& event);
	void transHandlerStand2FollowObject(const EventFsmSuper& event);
	void transHandlerStand2LieDown(const EventFsmSuper& event);
	void transHandlerStand2Run(const EventFsmSuper& event);
	void transHandlerStand2Talk(const EventFsmSuper& event);
	void transHandlerStand2Teleop(const EventFsmSuper& event);
	void transHandlerMoveToGoal2Stand(const EventFsmSuper& event);
	void transHandlerMoveAround2Stand(const EventFsmSuper& event);
	void transHandlerFollowObject2Stand(const EventFsmSuper& event);
	void transHandlerLieDown2Stand(const EventFsmSuper& event);
	void transHandlerRun2Stand(const EventFsmSuper& event);
	void transHandlerTalk2Stand(const EventFsmSuper& event);
	void transHandlerTeleop2Stand(const EventFsmSuper& event);

	/** @} */ // end of transition handlers group

	/**
	 * @brief Transition table order also defines transition priority
	 * @details Guard returning true allows transition between states to happen.
	 * @note mem_fn_row will likely change after switching to C++17
	 */
	using transition_table = table<
		mem_fn_row<STAND, EventFsmSuper, MOVE_TO_GOAL, &FsmSuper::transHandlerStand2MoveToGoal, &FsmSuper::guardStand2MoveToGoal>,
		mem_fn_row<STAND, EventFsmSuper, MOVE_AROUND, &FsmSuper::transHandlerStand2MoveAround, &FsmSuper::guardStand2MoveAround>,
		mem_fn_row<STAND, EventFsmSuper, FOLLOW_OBJECT, &FsmSuper::transHandlerStand2FollowObject, &FsmSuper::guardStand2FollowObject>,
		mem_fn_row<STAND, EventFsmSuper, LIE_DOWN, &FsmSuper::transHandlerStand2LieDown, &FsmSuper::guardStand2LieDown>,
		mem_fn_row<STAND, EventFsmSuper, RUN, &FsmSuper::transHandlerStand2Run, &FsmSuper::guardStand2Run>,
		mem_fn_row<STAND, EventFsmSuper, TALK, &FsmSuper::transHandlerStand2Talk, &FsmSuper::guardStand2Talk>,
		mem_fn_row<STAND, EventFsmSuper, TELEOP, &FsmSuper::transHandlerStand2Teleop, &FsmSuper::guardStand2Teleop>,

		mem_fn_row<MOVE_TO_GOAL, EventFsmSuper, STAND, &FsmSuper::transHandlerMoveToGoal2Stand, &FsmSuper::guardMoveToGoal2Stand>,
		mem_fn_row<MOVE_AROUND, EventFsmSuper, STAND, &FsmSuper::transHandlerMoveAround2Stand, &FsmSuper::guardMoveAround2Stand>,
		mem_fn_row<FOLLOW_OBJECT, EventFsmSuper, STAND, &FsmSuper::transHandlerFollowObject2Stand, &FsmSuper::guardFollowObject2Stand>,
		mem_fn_row<LIE_DOWN, EventFsmSuper, STAND, &FsmSuper::transHandlerLieDown2Stand, &FsmSuper::guardLieDown2Stand>,
		mem_fn_row<RUN, EventFsmSuper, STAND, &FsmSuper::transHandlerRun2Stand, &FsmSuper::guardRun2Stand>,
		mem_fn_row<TALK, EventFsmSuper, STAND, &FsmSuper::transHandlerTalk2Stand, &FsmSuper::guardTalk2Stand>,
		mem_fn_row<TELEOP, EventFsmSuper, STAND, &FsmSuper::transHandlerTeleop2Stand, &FsmSuper::guardTeleop2Stand>

	>;

	friend class fsmlite::fsm<FsmSuper>;
};

} // namespace hubero
