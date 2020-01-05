/*
 * Action.h
 *
 *  Created on: Dec 31, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_ACTION_H_
#define INCLUDE_ACTOR_CORE_ACTION_H_

#include <string>

namespace actor {
namespace core {

class Action {

public:

	/// \brief Default constructor
	Action(int status_terminal = FINISHED);

	/// \brief Resets the internal state
	/// \note Must be called on the action start routine
	void start(int status_initial = IN_PROGRESS);

	/// \brief Updates the status and the description
	/// \ref setStatus, see the template method placed below


	/// \brief `Brutal` method for state termination;
	/// after this call the \ref isTerminated method
	/// will immediately return true
	void terminate();

	/// \brief Returns the current status
	int getStatus() const;

	/// \brief Returns the status description
	std::string getStatusDescription() const;

	/// \brief Evaluates whether the state should finish by checking
	/// the current status and terminal status equality
	/// along with the \ref terminate_ flag
	bool isTerminated() const;

	/// \brief Destructor
	virtual ~Action();

private:

	/// \brief Stores the current status of the action
	int status_;

	/// \brief Stores the terminal status of the action
	int status_terminal_;

	/// \brief Flag risen when the state was manually terminated via \ref terminate call
	bool terminate_;

	/// \brief Stores a potentially useful in debugging status description
	std::string text_;

public:

	/// \brief Updates the status and the description regardless of enum class `T`
	template <typename T>
	void setStatus(T status, const std::string &description = "") {
		status_ = static_cast<int>(status);
		text_ = description;
	}

public:

	/// \section Scoped enumerations
	/// \brief Enum constants related to multiple tasks
	typedef enum {
		UNKNOWN = 9999u,	 //!< UNKNOWN: initial state
		IN_PROGRESS = 9998u, //!< IN_PROGRESS: a generic state for actions that consist of an atomic task (that are `non-dividable`)
		FINISHED = 9997u,    //!< FINISHED
		WRONG_STATE = 9996u, //!< WRONG_STATE: usually active when a previous state was not terminated and the new one was requested to start
		ABORTED = 9995u      //!< ABORTED: active when the action cannot be processed (for some reason)
	} SharedStatuses;

	/// \brief FollowObject PTF statuses. A proper status number will be set as a status ID
	/// by the FollowObject ROS Action Server
	enum class FollowObjectStatus {
		UNABLE_TO_FIND_PLAN,  //!< UNABLE_TO_FIND_PLAN
		TRACKING,             //!< TRACKING: the actor is actually following an object (i.e. the object is moving)
		OBJECT_REACHED,       //!< TARGET_REACHED: the followed object is close enough to the actor and the object is static
		ROTATE_TOWARDS_OBJECT,//!< ROTATE_TOWARDS_OBJECT
		NON_REACHABLE,        //!< NON_REACHABLE: the followed object is located in the place considered as a lethal obstacle on the actor costmap
		WAIT_FOR_MOVEMENT,    //!< WAIT_FOR_MOVEMENT
	};

	/// \brief Statuses expressed as enum constants
	/// associated with `SetGoal`/`TargetReaching` state
	enum class SetGoalStatus {
		FOLLOWING,            //!< FOLLOWING
		APPROACHING,          //!< APPROACHING: associated with the movement to the goal direction (\ref actor::core::Actor::manageTargetSingleReachment); may be used by many tasks
		ROTATE_TOWARDS_OBJECT,//!< ROTATE_TOWARDS_OBJECT: associated with alignToTargetDirection `state`
		GOAL_NOT_SELECTED,    //!< GOAL_NOT_SELECTED
		GOAL_REACHED,         //!< GOAL_REACHED
		NON_REACHABLE         //!< NON_REACHABLE
	};

	/// \brief Statuses expressed as enum constants
	/// associated with `LieDown` state
	enum class LieDownStatus {
		OBJECT_NON_REACHABLE, //!< OBJECT_NON_REACHABLE
		ROTATE_TOWARDS_OBJECT,//!< ROTATE_TOWARDS_OBJECT: associated with alignToTargetDirection `state`
		APPROACHING,          //!< APPROACHING
		LYING,                //!< LYING: used in the `lie_down` state, associated with the internal `idle` state
		STANDING_UP,          //!< STANDING_UP: used in the `lie_down` state, enabled after state termination request
		FINISHED              //!< FINISHED
	};

	/// \brief Statuses expressed as enum constants
	/// associated with `MoveAround` state
	enum class MovingAroundStatus {
		APPROACHING,          //!< APPROACHING
		ROTATE_TOWARDS_OBJECT,//!< ROTATE_TOWARDS_OBJECT: associated with alignToTargetDirection `state`
		GOAL_REACHED,         //!< GOAL_REACHED
		FAILED                //!< FAILED
	};

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_ACTION_H_ */
