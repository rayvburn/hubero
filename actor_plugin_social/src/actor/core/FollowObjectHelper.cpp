/*
 * FollowObjectHelper.cpp
 *
 *  Created on: Jan 5, 2020
 *      Author: rayvburn
 */

#include <actor/core/FollowObjectHelper.h>

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

FollowObjectHelper::FollowObjectHelper(): path_calc_tries_num_(10) {
}

// ------------------------------------------------------------------- //

void FollowObjectHelper::start() {
	path_calc_tries_num_ = 10;
}

// ------------------------------------------------------------------- //

bool FollowObjectHelper::doWait(const gazebo::common::Time &time_curr) {

		if ( (time_curr - time_last_path_calculation_).Double() < 2.0 ) {
			return (true);
		}

		// time update
		time_last_path_calculation_ = time_curr;

		if ( path_calc_tries_num_-- ) {
			return (true);
		} else {
			// enough...
			return (false);
		}

}

// ------------------------------------------------------------------- //

FollowObjectHelper::~FollowObjectHelper() {}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
