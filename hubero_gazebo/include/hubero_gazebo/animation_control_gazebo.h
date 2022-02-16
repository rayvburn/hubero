#pragma once

#include <hubero_interfaces/animation_control_base.h>
#include <gazebo/physics/Actor.hh>

#include <map>
#include <string>

namespace hubero {

class AnimationControlGazebo: public AnimationControlBase {
public:
    AnimationControlGazebo();

    void initialize(
        std::function<void(gazebo::physics::TrajectoryInfoPtr&)> anim_updater,
        const gazebo::physics::Actor::SkeletonAnimation_M& anims,
        const AnimationType& anim_init,
        const double& standing_height
    );

    virtual void adjustPose(Pose3& pose, const Time& time_current) override;

    inline gazebo::physics::TrajectoryInfoPtr& getTrajectoryInfo() {
        return trajectory_info_ptr_;
    }

protected:
    /**
     * @brief Helper for conversion of AnimationType to associated animation name (string)
     * @note This is simulator-specific, therefore must be defined in a derived class
     */
    static const std::map<AnimationType, std::string> animation_name_map_;

    void setupAnimation(AnimationType animation_type);

    void handlerStand();
    void handlerWalk();
    void handlerLieDown();
    void handlerLying();
    void handlerSitDown();
    void handlerSitting();
    void handlerStandUp();
    void handlerRun();
    void handlerTalk();

    /// Handy for catching initial height
    bool animation_configured_recently_;

    /// Initial pose of the actor with the current animation
    Pose3 animation_pose_initial_;

    /// Height of the actor (Z component of position) while he is standing still
    double standing_height_;

    /// A dictionary of skeleton animations
    gazebo::physics::Actor::SkeletonAnimation_M skeleton_anims_;

    /// Custom trajectory info
    gazebo::physics::TrajectoryInfoPtr trajectory_info_ptr_;

    /// Functor that takes trajectory info and updates animation in the simulator
    std::function<void(gazebo::physics::TrajectoryInfoPtr&)> trajectory_updater_;

}; // class AnimationControlGazebo

} // namespace hubero
