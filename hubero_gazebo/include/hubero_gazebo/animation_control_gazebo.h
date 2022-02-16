#pragma once

#include <hubero_interfaces/animation_control_base.h>
#include <gazebo/physics/Actor.hh>

#include <map>
#include <string>

namespace hubero {

class AnimationControlGazebo: public AnimationControlBase {
public:
    /**
     * @defgroup actorplugin Hard-coded values to carry animation transitions for Gazebo ActorPlugin characters
     * // FIXME
     */
    /// Defines typical position.z offset from standing height while lying on the ground
    const double POS_Z_LYING_GROUND_DELTA = -0.9;

    /// Defines typical position.z offset from standing height while sitting with feet touching the ground
    const double POS_Z_SITTING_DELTA = -0.25;

    /// Defines roll angle of orientation component while actor is lying on the ground
    const double ROT_ROLL_LYING_GROUND = -IGN_PI / 2;

    /// Defines roll angle of orientation component while actor is standing straight
    const double ROT_ROLL_STANDING = 0.0;

    /// @}

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

    /// Handy for catching initial pose
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
