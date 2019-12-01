/*
 * FrameGlobal.h
 *
 *  Created on: Dec 1, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_FRAMEGLOBAL_H_
#define INCLUDE_ACTOR_FRAMEGLOBAL_H_

#include <string>

namespace actor {

class FrameGlobal {

public:

	/// @brief Default constructor
	FrameGlobal();

	/// @brief Non-static method so only classes containing FrameGlobal instance
	/// can change the frame. In this system only actor::core::Actor provides
	/// such feature.
	void setFrame(const std::string &name);

	/// @brief Global frame getter method.
	/// @details Static method.
	static std::string getFrame();

	/// @brief Default destructor
	virtual ~FrameGlobal();

private:

	/// @brief Stores a name of the global frame for actors.
	static std::string frame_global_;

};

} /* namespace actor */

#endif /* INCLUDE_ACTOR_FRAMEGLOBAL_H_ */
