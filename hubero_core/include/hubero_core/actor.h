#pragma once

#include <hubero_interfaces/navigation_base.h>
#include <hubero_interfaces/localisation_base.h>
#include <hubero_interfaces/task_request_base.h>
#include <hubero_interfaces/animation_control_base.h>
#include <hubero_interfaces/model_control_base.h>

#include <hubero_common/defines.h>
#include <hubero_common/time.h>
#include <hubero_common/typedefs.h>

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
	void bbAlignToTarget();
	void bbMoveToGoal();
	/// @}

	/**
	 * @defgroup Interface classes
	 * `boost` shared_ptr used due to the fact that `pluginlib` uses it instead of `std` library
	 * @{
	 */
	boost::shared_ptr<hubero::NavigationBase> navigation_ptr_;
	boost::shared_ptr<hubero::LocalisationBase> localisation_ptr_;
	boost::shared_ptr<hubero::TaskRequestBase> task_req_ptr_;
	boost::shared_ptr<hubero::AnimationControlBase> animation_control_ptr_;
	boost::shared_ptr<hubero::ModelControlBase> model_control_ptr_;
	/// @}

	// FSM w actor cs, ten FSM bedzie zawieral FSMy poszczególnych zadan
}

} // namespace hubero
