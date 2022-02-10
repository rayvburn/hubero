#include <hubero_gazebo/world_geometry_gazebo.h>
#include <hubero_common/logger.h>

namespace hubero {

std::map<std::string, ModelGeometry> WorldGeometryGazebo::world_actor_data_;

WorldGeometryGazebo::WorldGeometryGazebo(): WorldGeometryBase::WorldGeometryBase() {}

void WorldGeometryGazebo::initialize(const std::string& world_frame_id) {
    HUBERO_LOG("[WorldGeometryGazebo] use Gazebo version of 'initialize' method!\r\n");
}

void WorldGeometryGazebo::initialize(
    const std::string& world_frame_id,
    const gazebo::physics::WorldPtr& world_ptr,
    const std::string& actor_name
) {
    world_ptr_ = world_ptr;
    actor_name_ = actor_name;
    WorldGeometryGazebo::world_actor_data_.insert({actor_name, ModelGeometry(actor_name)});
    WorldGeometryBase::initialize(world_frame_id);
}

void WorldGeometryGazebo::updateActor(
    const Pose3& pose,
    const Vector3& vel_ang,
    const Vector3& vel_lin,
    const Vector3& acc_ang,
    const Vector3& acc_lin,
    const BBox& box
) {
    auto it = WorldGeometryGazebo::world_actor_data_.find(actor_name_);
    if (it == WorldGeometryGazebo::world_actor_data_.end()) {
        HUBERO_LOG("[WorldGeometryGazebo] Cannot find '%s' actor name in map\r\n", actor_name_.c_str());
        return;
    }
    it->second = ModelGeometry(actor_name_, WorldGeometryBase::getFrame(), pose, vel_ang, vel_lin, acc_ang, acc_lin, box);
}

ModelGeometry WorldGeometryGazebo::getModel(const std::string& name) const {
    auto it = WorldGeometryGazebo::world_actor_data_.find(name);
    if (it != WorldGeometryGazebo::world_actor_data_.end()) {
        return it->second;
    }
    return getModel(world_ptr_->ModelByName(name));
}

ModelGeometry WorldGeometryGazebo::getModel(const gazebo::physics::ModelPtr& model_ptr) const {
    return ModelGeometry(
        actor_name_,
        WorldGeometryBase::getFrame(),
        model_ptr->WorldPose(),
        model_ptr->WorldLinearVel(),
        model_ptr->WorldAngularVel(),
        model_ptr->WorldLinearAccel(),
        model_ptr->WorldAngularAccel(),
        model_ptr->BoundingBox()
    );
}

} // namespace hubero