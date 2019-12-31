/*
 * Action.cpp
 *
 *  Created on: Dec 31, 2019
 *      Author: rayvburn
 */

#include <actor/core/Action.h>

namespace actor {
namespace core {

Action::Action(): status_(UNKNOWN) {}

void Action::setStatus(ActionStatus status) {
	status_ = status;
}

Action::ActionStatus Action::getStatus() const {
	return (status_);
}

Action::~Action() {}

} /* namespace core */
} /* namespace actor */
