#pragma once

#include <hubero_interfaces/animation_control_base.h>
#include <gazebo/physics/Actor.hh>

#include <map>
#include <string>

namespace hubero {

class AnimationControlGazebo: public AnimationControlBase {
public:
    AnimationControlGazebo(
        const gazebo::physics::Actor::SkeletonAnimation_M& anims,
        const AnimationType& anim_init
    );

    virtual void adjustPose(Pose3& pose, const Time& time_current) const override;

    gazebo::physics::TrajectoryInfoPtr& getTrajectoryInfo() const {
        return trajectory_info_ptr_;
    }

protected:
    /**
     * @brief Helper for conversion of AnimationType to associated animation name (string)
     * @note This is simulator-specific, therefore must be defined in a derived class
     */
    static const std::map<AnimationType, std::string> animation_name_map_;

    void setupAnimation();

    void handlerStand();
    void handlerWalk();
    void handlerLieDown();
    void handlerSitDown();
    void handlerSitting();
    void handlerStandUp();
    void handlerRun();
    void handlerTalk();

    /// Handy for catching initial height
    bool animation_configured_recently_;

    /// Initial height of the actor with the current animation
    double animation_height_initial_;

    /// A dictionary of skeleton animations
    gazebo::physics::Actor::SkeletonAnimation_M skeleton_anims_;

    /// Custom trajectory info
    gazebo::physics::TrajectoryInfoPtr trajectory_info_ptr_;

}; // class AnimationControlGazebo

} // namespace hubero
