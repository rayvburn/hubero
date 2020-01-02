/*
 * SingleGoal.h
 *
 *  Created on: Jan 2, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_SINGLEGOAL_H_
#define INCLUDE_ACTOR_CORE_SINGLEGOAL_H_

#include <actor/core/PTF.h>

namespace actor {
namespace core {

class SingleGoal : public PTF {

public:

	SingleGoal();

	virtual void execute() override;


	virtual ~SingleGoal();

private:



};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_SINGLEGOAL_H_ */
