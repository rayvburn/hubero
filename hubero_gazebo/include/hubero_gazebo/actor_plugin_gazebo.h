/**
 * @file actor_plugin_gazebo.h
 * @author Jarosław Karwowski (chromedivizer@gmail.com)
 * @brief Based on https://github.com/osrf/gazebo/blob/master/plugins/ActorPlugin.hh
 */
#pragma once

#include <string>
#include <vector>
#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

#include <sdf/sdf.hh>

#include <hubero_gazebo/animation_control_gazebo.h>
#include <hubero_gazebo/localisation_gazebo.h>
#include <hubero_gazebo/model_control_gazebo.h>
#include <hubero_gazebo/time_gazebo.h>
#include <hubero_gazebo/world_geometry_gazebo.h>

#include <hubero_ros/node.h>
#include <hubero_ros/task_request_ros.h>
#include <hubero_ros/navigation_ros.h>

#include <hubero_core/actor.h>

namespace gazebo {

class GAZEBO_VISIBLE ActorPlugin: public ModelPlugin {
public:
	/// \brief Constructor
	ActorPlugin();

	/// \brief Load the actor plugin.
	/// \param[in] model Pointer to the parent model.
	/// \param[in] sdf Pointer to the plugin's SDF elements.
	virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

	// Documentation Unherited.
	virtual void Reset();

protected:
	void initializeHuberoSimInterface();

	bool controller_enabled_;

	hubero::Actor hubero_actor_;
	std::shared_ptr<hubero::Node> ros_node_ptr_;
	// TODO: _ptr_
	std::shared_ptr<hubero::AnimationControlGazebo> sim_animation_control_;
	std::shared_ptr<hubero::LocalisationGazebo> sim_localisation_;
	std::shared_ptr<hubero::ModelControlGazebo> sim_model_control_;
	std::shared_ptr<hubero::WorldGeometryGazebo> sim_world_geometry_;

	std::shared_ptr<hubero::TaskRequestRos> task_;
	// std::shared_ptr<hubero::NavigationRos> nav_;

private:
	/// \brief Function that is called every update cycle.
	/// \param[in] info Timing information
	void OnUpdate(const common::UpdateInfo& info);

	/// \brief Pointer to the model.
	physics::ModelPtr model_ptr_;

	/// \brief Pointer to the parent actor.
	physics::ActorPtr actor_ptr_;

	/// \brief Pointer to the world, for convenience.
	physics::WorldPtr world_ptr_;

	/// \brief Pointer to the sdf element.
	sdf::ElementPtr sdf_ptr_;

	/// \brief List of connections
	std::vector<event::ConnectionPtr> connections_;
}; // class ActorPlugin
} // namespace gazebo
