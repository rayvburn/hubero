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

void Action::setStatus(ActionStatus status, const std::string &description) {
	status_ = status;
	text_ = description;
}

Action::ActionStatus Action::getStatus() const {
	return (status_);
}

std::string Action::getStatusDescription() const {
	return (text_);
}

Action::~Action() {}

} /* namespace core */
} /* namespace actor */
