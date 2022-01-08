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
#include <hubero_core/fsm_super.h>

#include <memory>

namespace hubero {

/**
 * @brief Control subsystem of the Actor agent
 */
class Actor {
public:
	Actor();

	void initialize(const std::string& agent_name);

	void update(const Time& time);

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

	/// Highest level Finite State Machine that orchestrates Actor activities
	FsmSuper fsm_;

	/**
	 * @defgroup Task classes that orchestrate specific tasks
	 * @note Tasks stored as shared_ptr to pass them to TaskRequest class
	 * @{
	 */
	std::shared_ptr<TaskStand> task_stand_ptr_;

	/**
	 * @defgroup Interface classes
	 * @{
	 */
	/*
	std::shared_ptr<hubero::NavigationBase> navigation_ptr_;
	std::shared_ptr<hubero::LocalisationBase> localisation_ptr_;
	std::shared_ptr<hubero::TaskRequestBase> task_req_ptr_;
	std::shared_ptr<hubero::AnimationControlBase> animation_control_ptr_;
	std::shared_ptr<hubero::ModelControlBase> model_control_ptr_;
	*/
	/// @}
}; // class Actor

} // namespace hubero
