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
#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include <sdf/sdf.hh>

#include <hubero_gazebo/animation_control_gazebo.h>
#include <hubero_gazebo/localisation_gazebo.h>
#include <hubero_gazebo/model_control_gazebo.h>
#include <hubero_gazebo/time_gazebo.h>
#include <hubero_gazebo/world_geometry_gazebo.h>

namespace gazebo {

class GAZEBO_VISIBLE ActorPlugin: public ModelPlugin {
public:
	/// \brief Constructor
	ActorPlugin();

	/// \brief Load the actor plugin.
	/// \param[in] _model Pointer to the parent model.
	/// \param[in] _sdf Pointer to the plugin's SDF elements.
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

	// Documentation Unherited.
	virtual void Reset();

protected:
	/*
	/// \brief Actor class object
	std::shared_ptr<actor::core::Actor> actor_ptr_;

	/// \brief List of models to ignore. Used for vector field
	std::vector<std::string> ignoreModels;

	bool controller_enabled_;

	std::shared_ptr<hubero::interface::LocalisationBase> localisation_ptr_;
	*/

	// std::shared_ptr<hubero::AnimationControlBase> sim_animation_control_;
	// std::shared_ptr<hubero::LocalisationBase> sim_localisation_;
	// std::shared_ptr<hubero::ModelControlBase> sim_model_control_;
	// std::shared_ptr<hubero::WorldGeometryBase> sim_world_geometry_;
	// std::shared_ptr<hubero::TimeBase> sim_time_;

private:
	/// \brief Function that is called every update cycle.
	/// \param[in] _info Timing information
	void OnUpdate(const common::UpdateInfo &_info);

	/// \brief Pointer to the model.
	physics::ModelPtr model;

	/// \brief Pointer to the parent actor.
	physics::ActorPtr actor;

	/// \brief Pointer to the world, for convenience.
	physics::WorldPtr world;

	/// \brief Pointer to the sdf element.
	sdf::ElementPtr sdf;

	/// \brief List of connections
	std::vector<event::ConnectionPtr> connections;
}; // class ActorPlugin
} // namespace gazebo
#endif
