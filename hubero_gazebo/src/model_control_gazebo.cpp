#include <hubero_gazebo/model_control_gazebo.h>
#include <functional>
#include <gazebo/physics/Link.hh>

namespace hubero {

ModelControlGazebo::ModelControlGazebo(): ModelControlBase::ModelControlBase() {}

void ModelControlGazebo::initialize(gazebo::physics::ActorPtr& actor_ptr, const std::string& frame_id) {
    // for simplicity
    typedef gazebo::physics::Actor GazeboActor;
    typedef gazebo::physics::Link GazeboLink;

    // NOTE: explicit ignition::math objects given here
    // NOTE: 2 superfluous arguments are hard-coded here
    auto fun_world_pose = std::bind(
        static_cast<void(GazeboActor::*)(const Pose3&, const bool, const bool)>(&GazeboActor::SetWorldPose),
        actor_ptr,
        std::placeholders::_1, true, true
    );
    auto fun_linear_vel = std::bind(
        static_cast<void(GazeboActor::*)(const Vector3&)>(&GazeboActor::SetLinearVel),
        actor_ptr,
        std::placeholders::_1
    );
    auto fun_angular_vel = std::bind(
        static_cast<void(GazeboActor::*)(const Vector3&)>(&GazeboActor::SetAngularVel),
        actor_ptr,
        std::placeholders::_1
    );

    /*
     * Previously GazeboActor::SetLinearAccel and GazeboActor::SetAngularAccel were called but newer Gazebo warns:
     *   [Wrn] [Model.cc:732] Model::SetAngularAccel() is deprecated and has no effect.
     *   Use Link::SetTorque() on the link directly instead.
     */
    // Virtual human body skeleton - link name is standardized
    auto hips_link = actor_ptr->GetLink("Hips");
    /*
     * Linear acceleration (force) and angular acceleration (torque) will likely not affect character
     * based on Gazebo's ActorPlugin since the plugin does not handle dynamics in any kind
     */
    auto fun_linear_accel = std::bind(
        static_cast<void(GazeboLink::*)(const Vector3&)>(&GazeboLink::SetForce),
        hips_link,
        std::placeholders::_1
    );
    // angular acceleration - torque is updated here
    auto fun_angular_accel = std::bind(
        static_cast<void(GazeboLink::*)(const Vector3&)>(&GazeboLink::SetTorque),
        hips_link,
        std::placeholders::_1
    );

    ModelControlBase::initialize(
        frame_id,
        fun_world_pose,
        fun_linear_vel,
        fun_angular_vel,
        fun_linear_accel,
        fun_angular_accel
    );
}

} // namespace hubero
