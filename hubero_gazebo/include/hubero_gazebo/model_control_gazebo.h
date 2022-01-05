#pragma once

#include <gazebo/physics/Actor.hh>
#include <hubero_interfaces/model_control_base.h>

namespace hubero {

class ModelControlGazebo: public ModelControlBase {
public:
    ModelControlGazebo(const std::string& frame_id);

    void initialize(gazebo::physics::ActorPtr& actor_ptr);
}; // class ModelControlGazebo

} // namespace hubero
