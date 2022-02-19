#include <hubero_gazebo/model_control_gazebo.h>
#include <functional>

namespace hubero {

ModelControlGazebo::ModelControlGazebo(): ModelControlBase::ModelControlBase() {}

void ModelControlGazebo::initialize(gazebo::physics::ActorPtr& actor_ptr, const std::string& frame_id) {
    // for simplicity
    typedef gazebo::physics::Actor GazeboActor;

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
    auto fun_linear_accel = std::bind(
        static_cast<void(GazeboActor::*)(const Vector3&)>(&GazeboActor::SetLinearAccel),
        actor_ptr,
        std::placeholders::_1
    );
    auto fun_angular_accel = std::bind(
        static_cast<void(GazeboActor::*)(const Vector3&)>(&GazeboActor::SetAngularAccel),
        actor_ptr,
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
