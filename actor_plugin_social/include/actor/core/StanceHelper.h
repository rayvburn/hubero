/*
 * StanceHelper.h
 *
 *  Created on: Mar 5, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_STANCEHELPER_H_
#define INCLUDE_ACTOR_CORE_STANCEHELPER_H_

#include "Enums.h"
#include <gazebo/physics/Actor.hh> // SkeletonAnimation_M
#include <queue>

namespace actor {
namespace core {

// FIXME: vector -> queue
class StanceHelper {

public:

	/// \brief Default constructor
	StanceHelper();

	/// \note Update the ActorPlugin trajectory with @ref getTrajectoryInfoPtr
	/// immediately after @ref init call to prevent these errors:
	/// [Err] [Actor.cc:582] Trajectory not found at time [X]"
	void init(const gazebo::physics::Actor::SkeletonAnimation_M &anims, const actor::ActorStance &stance_init);

	/// DEPRECATED, NEEDS TO BE ADJUSTED FOR HEIGHTS
	bool configure(const std::queue<actor::ActorStance> &stance_type_v, const gazebo::common::Time& time);

	bool configure(const actor::ActorStance &stance_type, const gazebo::common::Time& time, const double &height_init = 1.0);

	/// \brief Returns True if a new TrajectoryInfo was prepared and can be applied
	bool update(const gazebo::common::Time& time);

	void adjustStancePose(ignition::math::Pose3d &pose, const gazebo::common::Time& time);

	gazebo::physics::TrajectoryInfoPtr& getTrajectoryInfoPtr();

	actor::ActorStance getStance() const;

	/// \brief Prints stance update information in the terminal
	void printDebugInfo(const std::string &owner_name);

	/// \brief Helper function that converts a stance type to associated animation
	static std::string convertStanceToAnimationName(const actor::ActorStance &stance);

	static bool isDisposableAnimation(const actor::ActorStance &stance_type);

	virtual ~StanceHelper();

private:

	/// \brief Clears the @ref stance_sequence_ queue
	void clearQueue();

    /// \brief Actor's current stance
    actor::ActorStance stance_;

    /// \brief A sequence of consequent animations. This remains empty
    /// when last animation has been triggered, but the `stance_`
    /// stores the current stance of the actor
    std::queue<actor::ActorStance> stance_sequence_;

    double height_initial_;

    std::queue<double> height_sequence_;

    /// \brief A dictionary of skeleton animations
    gazebo::physics::Actor::SkeletonAnimation_M skeleton_anims_;

    /// \brief Custom trajectory info
    gazebo::physics::TrajectoryInfoPtr trajectory_info_ptr_;

    /// \brief Useful for animation transitions, indicates start time
	double trajectory_start_time_;

    /// \brief Useful for animation transitions, indicates end time
    double trajectory_end_time_;



    enum {
    	STANCE_UNKNOWN = 0u,	//!< STANCE_UNKNOWN: unknown state (initial)
    	STANCE_INITED,
    	STANCE_SELECTED,    	//!< STANCE_SELECTED: some stance was selected but is not marked as executed yet
		STANCE_EXECUTION,      	//!< STANCE_EXECUTION: some stance is currently executed
		STANCE_TYPE_UNKNOWN = 9999u
    };

    /// \brief Status of the stance management. Can be a value of the @ref enum
    uint8_t stance_status_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_STANCEHELPER_H_ */
