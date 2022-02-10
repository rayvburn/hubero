#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/time.h>
#include <hubero_common/typedefs.h>

#include <hubero_interfaces/navigation_base.h>
#include <hubero_interfaces/localisation_base.h>
#include <hubero_interfaces/task_request_base.h>
#include <hubero_interfaces/animation_control_base.h>
#include <hubero_interfaces/model_control_base.h>
#include <hubero_interfaces/world_geometry_base.h>

#include <hubero_core/internal_memory.h>

#include <hubero_core/tasks/task_stand.h>
#include <hubero_core/tasks/task_move_to_goal.h>
#include <hubero_core/tasks/task_move_around.h>
#include <hubero_core/tasks/task_lie_down.h>
#include <hubero_core/tasks/task_sit_down.h>
#include <hubero_core/tasks/task_follow_object.h>
#include <hubero_core/tasks/task_teleop.h>
#include <hubero_core/tasks/task_run.h>
#include <hubero_core/tasks/task_talk.h>

#include <hubero_core/fsm/fsm_super.h>

#include <memory>

namespace hubero {

/**
 * @brief Control subsystem of the Actor agent
 * @details Geometric data uses the same coordinate system as the simulator does
 */
class Actor {
public:
	Actor();

	void initialize(
		const std::string& actor_sim_name,
		std::shared_ptr<hubero::AnimationControlBase> animation_control_ptr,
		std::shared_ptr<hubero::ModelControlBase> model_control_ptr,
		std::shared_ptr<hubero::WorldGeometryBase> world_geometry_ptr,
		std::shared_ptr<hubero::LocalisationBase> localisation_ptr,
		std::shared_ptr<hubero::NavigationBase> navigation_ptr,
		std::shared_ptr<hubero::TaskRequestBase> task_request_ptr
	);

	void update(const Time& time);

	bool isInitialized() const;

	/**
	 * @brief Adds transition handlers for FsmSuper
	 * @details Transition handlers aim to update tasks state so tasks can properly report their state as active
	 * or finished
	 * @details Static method in Actor instead of making tasks arguments to @ref FsmSuper class to avoid circular
	 * dependency
	 */
	static void addFsmSuperTransitionHandlers(
		FsmSuper& fsm,
		std::shared_ptr<TaskStand> task_stand_ptr,
		std::shared_ptr<TaskMoveToGoal> task_move_to_goal_ptr,
		std::shared_ptr<TaskMoveAround> task_move_around_ptr,
		std::shared_ptr<TaskLieDown> task_lie_down_ptr,
		std::shared_ptr<TaskSitDown> task_sit_down_ptr,
		std::shared_ptr<TaskFollowObject> task_follow_object_ptr,
		std::shared_ptr<TaskTeleop> task_teleop_ptr,
		std::shared_ptr<TaskRun> task_run_ptr,
		std::shared_ptr<TaskTalk> task_talk_ptr,
		std::shared_ptr<NavigationBase> navigation_ptr
	);

	/**
	 * @brief Returns displacement (in meters) made in the most recent update
	 */
	inline double getDisplacement() const {
		return mem_.getDisplacement();
	}

protected:
	/**
	 * @defgroup taskhelpers Task helper methods
	 *
	 * Methods provide basic behaviour execution (based on task FSM definition)
	 */
	/// @brief Prepare FSM predicates update and executes appropriate basic behaviour of an active task
	void executeTaskStand();
	void executeTaskMoveToGoal();
	void executeTaskMoveAround();
	void executeTaskLieDown();
	void executeTaskSitDown();
	void executeTaskFollowObject();
	void executeTaskTeleop();
	void executeTaskRun();
	void executeTaskTalk();

	/// @brief Terminates all tasks (their predicates) except given TaskType
	void terminateOtherTasks(TaskType task_type_current);

	/// @}

	/**
	 * @defgroup bbs Basic behaviour methods
	 * @{
	 */
	void bbStand();
	void bbAlignToTarget();
	void bbMoveToGoal();
	void bbChooseNewGoal();
	void bbAwaitObjectMovement();
	void bbLieDown();
	void bbStandUpFromLying();
	void bbSitDown();
	void bbStandUpFromSitting();
	void bbRun();
	void bbTalk();
	void bbTeleop();

	/// @}

	/**
	 * @defgroup bbhelpers Basic behaviour helper methods
	 * @details Methods usually used to setup some actions at task startup
	 * @{
	 */
	void prepareNavigationWalk();

	/// @}

	/// @brief Updates predicates of the highest level Finite State Machine
	void updateFsmSuper();

	/// Name of the actor in simulator
	std::string actor_sim_name_;

	/// @brief Contains essential values of internal memory of the control subsystem
	InternalMemory mem_;

	/// Highest level Finite State Machine that orchestrates Actor tasks
	FsmSuper fsm_;

	/**
	 * @defgroup taskclass Task classes that orchestrate specific tasks
	 * @note Tasks stored as shared_ptr to pass them to TaskRequest class
	 * @{
	 */
	std::shared_ptr<TaskStand> task_stand_ptr_;
	std::shared_ptr<TaskMoveToGoal> task_move_to_goal_ptr_;
	std::shared_ptr<TaskMoveAround> task_move_around_ptr_;
	std::shared_ptr<TaskLieDown> task_lie_down_ptr_;
	std::shared_ptr<TaskSitDown> task_sit_down_ptr_;
	std::shared_ptr<TaskFollowObject> task_follow_object_ptr_;
	std::shared_ptr<TaskTeleop> task_teleop_ptr_;
	std::shared_ptr<TaskRun> task_run_ptr_;
	std::shared_ptr<TaskTalk> task_talk_ptr_;

	/// @}

	/**
	 * @defgroup maps Maps to prevent switch-case pattern while invoking calls of a currently active task
	 * @{
	 */
	/// @brief Map that bonds FSM states with internal memory updaters (this is specific to task/state)
	std::map<FsmSuper::State, std::function<void(InternalMemory&)>> state_memory_updater_map_;

	/// @brief Map that bonds FSM states with transition function executors (this is specific to task/state)
	std::map<FsmSuper::State, std::function<void(void)>> state_tf_exec_map_;

	/// @brief Map that bond TaskTypes with specific Task class
	std::map<TaskType, std::shared_ptr<TaskBase>> task_map_;

	/// @}

	/**
	 * @defgroup interface Interface classes
	 * @{
	 */
	std::shared_ptr<hubero::AnimationControlBase> animation_control_ptr_;
	std::shared_ptr<hubero::ModelControlBase> model_control_ptr_;
	std::shared_ptr<hubero::LocalisationBase> localisation_ptr_;
	std::shared_ptr<hubero::NavigationBase> navigation_ptr_;
	std::shared_ptr<hubero::TaskRequestBase> task_request_ptr_;
	std::shared_ptr<hubero::WorldGeometryBase> world_geometry_ptr_;

	/// @}

	/**
	 * @defgroup templates Template methods that must be defined in header file
	 * @{
	 */
	/**
	 * @brief Activates given task ptr, if necessary, fills up shared part of FSM event struct and returns that struct
	 *
	 * @tparam Tevent type of the FSM event struct
	 * @param task_ptr shared pointer to object that derives from TaskBase
	 */
	template <typename Tevent>
	Tevent prepareTaskFsmUpdate(const std::shared_ptr<TaskBase>& task_ptr) {
		// fill up task and navigation predicates - those are used for orchestration of task execution
		TaskPredicates task_predicates(*task_ptr);
		NavPredicates nav_predicates(navigation_ptr_->getFeedback());
		return Tevent(task_predicates, nav_predicates);
	}

	/// @} // end of templates group
}; // class Actor

} // namespace hubero
