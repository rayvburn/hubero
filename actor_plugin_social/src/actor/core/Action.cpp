/*
 * Action.cpp
 *
 *  Created on: Dec 31, 2019
 *      Author: rayvburn
 */

#include <actor/core/Action.h>

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

Action::Action(int status_terminal)
	: status_(UNKNOWN),
	  status_terminal_(status_terminal),
	  terminate_(false)
{}

// ------------------------------------------------------------------- //

void Action::start(int status_initial) {
	status_ = status_initial;
	text_ = "";
	terminate_ = false;
}

// ------------------------------------------------------------------- //

void Action::terminate() {
	terminate_ = true;
}

// ------------------------------------------------------------------- //

int Action::getStatus() const {
	return (status_);
}

// ------------------------------------------------------------------- //

std::string Action::getStatusDescription() const {
	return (text_);
}

// ------------------------------------------------------------------- //

bool Action::isTerminated() const {
	if ( terminate_ || (status_ == status_terminal_) ) {
		return (true);
	}
	return (false);
}

// ------------------------------------------------------------------- //

Action::~Action() {}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
