/*
 * PTF.h
 *
 *  Created on: Jan 2, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_PTF_H_
#define INCLUDE_ACTOR_CORE_PTF_H_

#include <actor/core/Target.h>
#include <memory>
#include <stdint.h>
#include <actor/core/Action.h>

namespace actor {
namespace core {

class PTF : public Action {

public:

	PTF();

	PTF(std::shared_ptr<actor::core::Target> target_manager_ptr);

	void initializeTargetManager(std::shared_ptr<actor::core::Target> target_manager_ptr);

	virtual void start();

	virtual void execute() = 0;

	bool isTerminalConditionFulfilled() const;

	virtual ~PTF();

protected:

	std::shared_ptr<actor::core::Target> target_manager_ptr_;

	bool terminal_flag_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_PTF_H_ */
