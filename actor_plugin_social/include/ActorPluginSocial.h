/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include "SocialForceModel.h"

namespace gazebo
{
  class GAZEBO_VISIBLE ActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Unherited.
    public: virtual void Reset();

    public: virtual ignition::math::Vector3d GetActorLinearVel();

    public: void TestSimActor(void);

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
    private: void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    private: void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Pointer to the model.
    private: physics::ModelPtr model; 								// ?? nothing earned with model use instead of actor - could be deleted...

    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief Desired velocity of the actor
    private: ignition::math::Vector3d velocity_desired;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    private: ignition::math::Vector3d target;

    /// \brief Target location weight (used for vector field)
    private: double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
    private: double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor = 1.0;

    /// \brief Time of the last update.
    private: common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    private: physics::TrajectoryInfoPtr trajectoryInfo;

    // ----------------------------------------------------------

    /// \brief Method that assigns an ID for the actor that is invoked by (must be called for each actor)
    private: static unsigned int InitActorInfo(const std::string &_name);

    /// \brief Actor's ID for indexing the lin_vels_vector (see below)
    private: unsigned int actor_id;

    /// \brief Linear velocity of the actor
    private: static std::vector<ignition::math::Vector3d> lin_vels_vector;

    private: static void SetActorsLinearVel(const unsigned int &_id, const ignition::math::Vector3d &_vel);

//    private: static std::map<std::string, unsigned int> map_of_names;





    /// \brief Linear velocity of the actor
    private: ignition::math::Vector3d linear_velocity; 	// ??????????????????

    /// \brief Last actor location
    private: ignition::math::Vector3d last_pos_actor;

    /// \brief Actual velocity of the actor
    private: ignition::math::Vector3d velocity_actual;

    /// \brief Helper function to calculate the velocity (if it is allowable)
    /// allowable in terms of immediate jumps which are not permitted
    private: bool CalculateVelocity(const ignition::math::Vector3d &_pos, const double &_dt);

    /// \brief Social Force Model interface object
    private: SocialForceModel::SocialForceModel sfm;


  };
}
#endif
