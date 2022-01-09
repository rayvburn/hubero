#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/time.h>
#include <hubero_common/typedefs.h>

#include <hubero_interfaces/navigation_base.h>
#include <hubero_interfaces/localisation_base.h>
#include <hubero_interfaces/task_request_base.h>
#include <hubero_interfaces/animation_control_base.h>
#include <hubero_interfaces/model_control_base.h>

#include <hubero_core/tasks/task_stand.h>
#include <hubero_core/tasks/task_move_to_goal.h>
#include <hubero_core/tasks/task_move_around.h>
#include <hubero_core/tasks/task_lie_down.h>
#include <hubero_core/tasks/task_sit_down.h>
#include <hubero_core/tasks/task_follow_object.h>
#include <hubero_core/tasks/task_teleop.h>
#include <hubero_core/tasks/task_run.h>
#include <hubero_core/tasks/task_talk.h>

#include <hubero_core/fsm_super.h>

#include <memory>

namespace hubero {

/**
 * @brief Control subsystem of the Actor agent
 */
class Actor {
public:
	Actor();

	void initializeSim(
		const std::string& actor_sim_name,
		std::shared_ptr<hubero::AnimationControlBase> animation_control_ptr,
		std::shared_ptr<hubero::ModelControlBase> model_control_ptr,
		std::shared_ptr<hubero::LocalisationBase> localisation_ptr
	);

	void initializeNav(std::shared_ptr<hubero::NavigationBase> navigation_ptr);

	void initializeTask(std::shared_ptr<hubero::TaskRequestBase> task_request_ptr);

	void update(const Time& time);

	bool isInitialized() const;

protected:
	/**
	 * @defgroup Basic behaviour methods
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

	/// Name of the actor in simulator
	std::string actor_sim_name_;

	/// Highest level Finite State Machine that orchestrates Actor activities
	FsmSuper fsm_;

	/**
	 * @defgroup Task classes that orchestrate specific tasks
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

	/**
	 * @defgroup Interface classes
	 * @{
	 */
	std::shared_ptr<hubero::AnimationControlBase> animation_control_ptr_;
	std::shared_ptr<hubero::ModelControlBase> model_control_ptr_;
	std::shared_ptr<hubero::LocalisationBase> localisation_ptr_;
	std::shared_ptr<hubero::NavigationBase> navigation_ptr_;
	std::shared_ptr<hubero::TaskRequestBase> task_request_ptr_;

	/// @}
}; // class Actor

} // namespace hubero
