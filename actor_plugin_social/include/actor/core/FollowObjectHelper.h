/*
 * FollowObjectHelper.h
 *
 *  Created on: Jan 5, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_FOLLOWOBJECTHELPER_H_
#define INCLUDE_ACTOR_CORE_FOLLOWOBJECTHELPER_H_

#include <gazebo/common/Time.hh>

namespace actor {
namespace core {

class FollowObjectHelper {

public:

	/// \brief Default constructor
	FollowObjectHelper();

	/// \brief Resets internal state
	void start();

    /// \brief Evaluates whether to skip the current iteration and wait
    /// for the tracked object to became available.
	/// \return False if the state should be terminated
	bool doWait(const gazebo::common::Time &time_curr);

	/// \brief Destructor
	virtual ~FollowObjectHelper();

private:

    /// \brief Stores the time of the last global path calculation while
    /// waiting for the tracked object to became achievable again
    gazebo::common::Time time_last_path_calculation_;

    /// \brief Counts number of tries of the global path calculation
    int path_calc_tries_num_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_FOLLOWOBJECTHELPER_H_ */
