/*
 * FrameGlobal.cpp
 *
 *  Created on: Dec 1, 2019
 *      Author: rayvburn
 */

#include <actor/FrameGlobal.h>

namespace actor {

// declaration of a static variable
std::string FrameGlobal::frame_global_;

FrameGlobal::FrameGlobal() {}

void FrameGlobal::setFrame(const std::string &name) {
	frame_global_ = name;
}

/// \details Static method.
std::string FrameGlobal::getFrame() {
	return (frame_global_);
}

FrameGlobal::~FrameGlobal() {}

} /* namespace actor */
