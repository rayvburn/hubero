#pragma once

#include <hubero_interfaces/world_geometry_base.h>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

namespace hubero {

class WorldGeometryGazebo: public WorldGeometryBase {
public:
    WorldGeometryGazebo();

    virtual void initialize(const std::string& world_frame_id) override;

    /**
     * @brief This method is Gazebo-specific
     */
    void initialize(const std::string& world_frame_id, gazebo::physics::WorldPtr& world_ptr, const std::string& actor_name);

    /**
     * @brief This method is Gazebo-specific, used for update of the actors velocities etc.
     */
    void updateActor(
        const Pose3& pose,
        const Vector3& vel_ang,
        const Vector3& vel_lin,
        const Vector3& acc_ang,
        const Vector3& acc_lin,
        const BBox& box
    );

	virtual ModelGeometry getModel(const std::string& name) override;

protected:
    ModelGeometry getModel(const gazebo::physics::ModelPtr& model_ptr);

    gazebo::physics::WorldPtr world_ptr_;

    /// Name of the actor that poses an instance of this class
    std::string actor_name_;

    /**
     * @brief This map is created as a workaround, see the link below for details
     * @details http://answers.gazebosim.org/question/22114/actor-related-information-in-gazebophysicsworldptr-and-collision-of-actors/
     */
    static std::map<std::string, ModelGeometry> world_actor_data_;
};

} // namespace hubero
