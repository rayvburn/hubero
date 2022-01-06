#pragma once

#include <gazebo/physics/Actor.hh>
#include <hubero_interfaces/model_control_base.h>

namespace hubero {

class ModelControlGazebo: public ModelControlBase {
public:
    ModelControlGazebo();

    void initialize(gazebo::physics::ActorPtr& actor_ptr, const std::string& frame_id);
protected:
    // cast is required due to 'error: no matching function for call to ‘bind(<unresolved overloaded function type>'
    
}; // class ModelControlGazebo

} // namespace hubero
