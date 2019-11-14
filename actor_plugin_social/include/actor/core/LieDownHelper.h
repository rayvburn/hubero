/*
 * LieDownHelper.h
 *
 *  Created on: Oct 9, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_LIEDOWNHELPER_H_
#define INCLUDE_ACTOR_CORE_LIEDOWNHELPER_H_

#include <ignition/math/Pose3.hh>


namespace actor {
namespace core {

/**
 * @brief Helper method storing meaningful data required during `LieDown` state operation.
 */
class LieDownHelper {

public:

	/**
	 * @brief Default constructor
	 */
	LieDownHelper();

	/**
	 * @brief Lying height is an actor position's `Z` coordinate
	 * while his stance is actually set to `LIE` see @ref Enums.
	 * @param height
	 */
	void setLyingHeight(const double &height);

	/**
	 * @brief Pose assigned to actor's model after reachment
	 * of his lying position.
	 * @param pose_center
	 */
	void setLyingPose(const ignition::math::Pose3d &pose_center);

	/**
	 * @brief Additional rotation applied to lying pose (see @ref setLyingPose)
	 * @param rot: rotation in radians
	 */
	void setRotation(const double &rot);

	/**
	 * @brief A pose which later on will be used at the exit
	 * from `LieDown` state. The pose must be a `safe place`
	 * in terms of costmap (i.e. low cost), otherwise
	 * actor will be stuck (unable to set next target, whole
	 * system must be reset then).
	 * @param pose
	 */
	void setPoseBeforeLying(const ignition::math::Pose3d &pose);

	/**
	 * @brief Sets internal state indicating whether
	 * an actor is currently lying.
	 * @param lied_down
	 */
	void setLying(const bool &lied_down);

	/**
	 * @brief Helper method which sets `do_finish_` flag to TRUE.
	 * This in turn indicates that `LieDown` state operation
	 * should be finished.
	 */
	void stopLying();

	/**
	 * @brief This method uses the pose given in @ref setPoseBeforeLying.
	 * Modifies it so actor's yaw angle is rotated PI/2 degrees -
	 * actor faces opposite direction at the exit.
	 * @return
	 */
	ignition::math::Pose3d computePoseFinishedLying() const;

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	/**
	 * @brief Returns lying down indicator set via @ref setLying call
	 * @return
	 */
	bool isLyingDown() const;

	/**
	 * @brief Returns height given during @ref setLyingHeight call
	 * @return
	 */
	double getLyingHeight() const;

	/**
	 * @brief Returns pose given during @ref setPoseBeforeLying call
	 * @return
	 */
	ignition::math::Pose3d getPoseBeforeLying() const;

	/**
	 * @brief Returns the pose required during lying based on the configuration
	 * provided by setter methods.
	 * @return
	 */
	ignition::math::Pose3d getPoseLying() const;

	/**
	 * @brief Evaluates if `LieDown` state stop is requested.
	 * @return
	 */
	bool doStopLying() const;

	/**
	 * @brief Destructor
	 */
	virtual ~LieDownHelper();

private:

    /// \brief Stores pose achieved in `LIE_DOWN` state
    /// just before actual lying (actor must approach
    /// to the target first).
    ignition::math::Pose3d pose_lie_start_;

    /// \brief Pose representing a center of the object
    /// actor has to lie down onto or just a position
    /// to lie down.
    ignition::math::Pose3d pose_center_;

    /// \brief Determines additional rotation around `Z` axis
    /// while lying. In fact it is not actual rotation around
    /// `Z` axis after actor has lied down.
    double rotation_;

    /// \brief Z component of the position during lying down.
    double pose_lie_height_;

    /// \brief Status
    bool is_lying_;

    /// \brief Indicator changes its state in `stopLying` call
    bool do_finish_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_LIEDOWNHELPER_H_ */
