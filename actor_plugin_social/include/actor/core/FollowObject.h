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

class FollowObject : public PTF {

public:

	typedef enum {

		UNKNOWN = 0,
		NOT_FOLLOWING,
		PATH_PLAN_CALCULATION,
		UNABLE_TO_FIND_PLAN,
		TRACKING,
		TARGET_REACHED,
		ROTATE_TOWARDS_OBJECT,
		NOT_REACHABLE,
		WAIT_FOR_MOVEMENT,
		FAILED,
		FINISHED

	} FollowObjectStatus;

	FollowObject();

	virtual void start() override;

	virtual void execute() override {}
	void execute(const gazebo::common::Time &curr_time);

	virtual ~FollowObject();

private:

	bool doWait(const gazebo::common::Time &curr_time);

	gazebo::common::Time time_last_path_calculation_;
	int path_calc_tries_num_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_FOLLOWOBJECT_H_ */
