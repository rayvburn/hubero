/*
 * FollowObject.h
 *
 *  Created on: Jan 2, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_FOLLOWOBJECT_H_
#define INCLUDE_ACTOR_CORE_FOLLOWOBJECT_H_

#include <actor/core/PTF.h>
#include <actor/core/Target.h>
#include <gazebo/common/Time.hh>

namespace actor {
namespace core {

/// \brief Manages FollowObject state operation
class FollowObject : public PTF {

public:

	/// \brief FollowObject PTF statuses. A proper status number will be set as a status ID
	/// by the FollowObject ROS Action Server
	typedef enum {
		UNKNOWN = 0,          //!< UNKNOWN
		NOT_FOLLOWING,        //!< NOT_FOLLOWING
		UNABLE_TO_FIND_PLAN,  //!< UNABLE_TO_FIND_PLAN
		TRACKING,             //!< TRACKING: the actor is actually following an object (i.e. the object is moving)
		OBJECT_REACHED,       //!< TARGET_REACHED: the followed object is close enough to the actor and the object is static
		ROTATE_TOWARDS_OBJECT,//!< ROTATE_TOWARDS_OBJECT
		NOT_REACHABLE,        //!< NOT_REACHABLE: the followed object is located in the place considered as a lethal obstacle on the actor costmap
		WAIT_FOR_MOVEMENT,    //!< WAIT_FOR_MOVEMENT
	} FollowObjectStatus;

	/// \brief Default constructor
	FollowObject();

	/// \brief Resets internal state of the PTF. Must be called
	/// at the startup of each action operation.
	virtual void start() override;

	/// \brief An empty method which must NOT be used.
	/// FIXME: an ugly solution
	virtual void execute() override {}

	/// \brief Performs partial transition function execution
	/// \param curr_time: current Gazebo simulation time
	void execute(const gazebo::common::Time &curr_time);

	/// \brief Destructor
	virtual ~FollowObject();

private:

	/// \brief Evaluates whether to skip the current iteration and wait
	/// for the tracked object to became available
	bool doWait(const gazebo::common::Time &curr_time);

	/// \brief Stores the time of the last global path calculation while
	/// waiting for the tracked object to became achievable again
	gazebo::common::Time time_last_path_calculation_;

	/// \brief Counts number of tries of the global path calculation
	int path_calc_tries_num_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_FOLLOWOBJECT_H_ */
