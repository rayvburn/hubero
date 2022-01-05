#include <hubero_gazebo/model_control_gazebo.h>
#include <functional>

namespace hubero {

ModelControlGazebo::ModelControlGazebo(const std::string& frame_id): ModelControlBase(frame_id) {}

void ModelControlGazebo::initialize(gazebo::physics::ActorPtr& actor_ptr) {
    ModelControlBase::initialize(
        std::bind(&gazebo::physics::Actor::SetWorldPose, actor_ptr, std::placeholders::__1),
        std::bind(&gazebo::physics::Actor::SetLinearVel, actor_ptr, std::placeholders::__1),
        std::bind(&gazebo::physics::Actor::SetAngularVel, actor_ptr, std::placeholders::__1),
        std::bind(&gazebo::physics::Actor::SetLinearAccel, actor_ptr, std::placeholders::__1),
        std::bind(&gazebo::physics::Actor::SetAngularAccel, actor_ptr, std::placeholders::__1)
    );
}

} // namespace hubero
