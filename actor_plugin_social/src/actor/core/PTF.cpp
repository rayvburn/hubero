/*
 * PTF.cpp
 *
 *  Created on: Jan 2, 2020
 *      Author: rayvburn
 */

#include <actor/core/PTF.h>

namespace actor {
namespace core {

PTF::PTF(): terminal_flag_(false) {}

PTF::PTF(std::shared_ptr<actor::core::Target> target_manager_ptr)
	: target_manager_ptr_(target_manager_ptr),
	  terminal_flag_(false)
{}

void PTF::initializeTargetManager(std::shared_ptr<actor::core::Target> target_manager_ptr) {
	target_manager_ptr_ = target_manager_ptr;
}

void PTF::start() {
	terminal_flag_ = false;
	this->status_int_ = 0;
	this->text_ = "";
}

bool PTF::isTerminalConditionFulfilled() const {
	return (terminal_flag_);
}

PTF::~PTF() {}

} /* namespace core */
} /* namespace actor */
