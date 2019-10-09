/*
 * LieDownHelper.h
 *
 *  Created on: Oct 9, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_CORE_LIEDOWNHELPER_H_
#define INCLUDE_CORE_LIEDOWNHELPER_H_

#include <ignition/math/Pose3.hh>


namespace actor {
namespace core {

class LieDownHelper {

public:

	LieDownHelper();

	void setNormalHeight(const double &height);
	void setLyingHeight(const double &height);
	//void setLyingPosition(const ignition::math::Vector3d &position); // coordinate-oriented cmd
	void setLyingPose(const ignition::math::Pose3d &pose_center);
	void setRotation(const double &rot);

	void setPoseBeforeLying(const ignition::math::Pose3d &pose);
	void setLying(const bool &lied_down);
	void stopLying();

	ignition::math::Pose3d computePoseFinishedLying() const;

	bool isLyingDown() const;
	double getNormalHeight() const;
	double getLyingHeight() const;
	ignition::math::Pose3d getPoseBeforeLying() const;
	ignition::math::Pose3d getPoseCenter() const;
	bool doStopLying() const;

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

    /// \brief Z component of the position before actual
    /// lying down (animation change).
    double pose_init_height_;

    /// \brief Z component of the position during lying down.
    double pose_lie_height_;

    /// \brief Status
    bool is_lying_;

    /// \brief Indicator changes its state in `stopLying` call
    bool do_finish_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_CORE_LIEDOWNHELPER_H_ */
