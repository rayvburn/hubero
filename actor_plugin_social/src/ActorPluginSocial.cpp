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

#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/physics/physics.hh"
#include "ActorPluginSocial.h"

GZ_REGISTER_MODEL_PLUGIN(gazebo::ActorPlugin)

/////////////////////////////////////////////////

gazebo::ActorPlugin::ActorPlugin(): controller_enabled_(false) { }

/////////////////////////////////////////////////
void gazebo::ActorPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

	this->sdf = _sdf;
	this->model = _model;
	this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
	this->world = this->actor->GetWorld();
	this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
		  std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

	// - - - - - - - - - - - - - - - - - - - - - -  - - - -- - - -- - - -- -  -- -

    std::cout << "LOADED POSE: " << this->actor->WorldPose() << std::endl;
	actor_ptr_ = std::make_shared<actor::core::Actor>();
	actor_ptr_->initGazeboInterface(actor, world);
	actor_ptr_->initRosInterface();
	actor_ptr_->initActor(_sdf);
	std::cout << "MODDED POSE: " << this->actor->WorldPose() << std::endl;

}

/////////////////////////////////////////////////
void gazebo::ActorPlugin::Reset()
{

}

/////////////////////////////////////////////////

void gazebo::ActorPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info)
{

	// NOTE: `_info.realTime.Double()` returns seconds from the launch
	//
	// Not waiting for the ROS to initialize produces further problems
	// (mainly with GetCostmapStatus service). The longest task in the
	// `actor_global_plan_node` is the costmap initialization which
	// somehow blocks processing callbacks from services (probably ros::spin()
	// is called after costmap's initialization and that creates this bad behaviour).
	// To check what did not help, check these commits:
	// https://github.com/rayvburn/gazebo_ros_people_sim/commit/efc52f04b2c10d764db78295e1a7f3c423a9c26e
	// https://github.com/rayvburn/gazebo_ros_people_sim/commit/62c8719b25aac5aff53447ba483045364fcd783f
	// After many tries brought back some delay as below:
	if ( !controller_enabled_ ) {
		if ( _info.realTime.Double() >= 2.5 ) {
			std::cout << "\t[ActorPlugin] Actor controller starting the job!" << std::endl;
			controller_enabled_ = true;
		}
		return;
	}

	// ???????????
	if ( !controller_enabled_ ) {
		SfmSetPrintData(false); // for some reason - REMOVING THIS completely FROM onUpdate makes Gazebo crash
		return;
	};

	actor_ptr_->executeTransitionFunction();

  	return; // OnUpdate testing

} /* OnUpdate */

// ===============================================================================================
