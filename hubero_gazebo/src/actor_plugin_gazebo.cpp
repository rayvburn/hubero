#include <hubero_gazebo/actor_plugin_gazebo.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::ActorPlugin)

namespace gazebo {

ActorPlugin::ActorPlugin():
	controller_enabled_(false),
	ros_node_ptr_(std::make_shared<hubero::Node>()),
	sim_animation_control_(std::make_shared<hubero::AnimationControlGazebo>()),
	sim_localisation_(std::make_shared<hubero::LocalisationGazebo>()),
	sim_model_control_(std::make_shared<hubero::ModelControlGazebo>()),
	sim_world_geometry_(std::make_shared<hubero::WorldGeometryGazebo>()),
	task_(std::make_shared<hubero::TaskRequestRos>())
{}

void ActorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
	/*
	 * Gazebo plugin-related setup
	 */
	sdf_ptr_ = sdf;
	model_ptr_ = model;
	actor_ptr_ = boost::dynamic_pointer_cast<physics::Actor>(model);
	world_ptr_ = actor_ptr_->GetWorld();
	connections_.push_back(event::Events::ConnectWorldUpdateBegin(
		std::bind(
			&ActorPlugin::OnUpdate,
			this,
			std::placeholders::_1)
		)
	);

	/*
	 * HuBeRo framework interface initialization
	 */
	initializeHuberoInterface();
	hubero_actor_.initializeSim(actor_ptr_->GetName(), sim_animation_control_, sim_model_control_, sim_localisation_);

	task_->initialize(ros_node_ptr_, actor_ptr_->GetName());

	/*
	 * Enable specific trajectory of actor
	 * It must be done in "Load", otherwise default animation (running) will be triggered and all actors in the 'world'
	 * will be located in exact the same place
	 */
	actor_ptr_->SetCustomTrajectory(sim_animation_control_->getTrajectoryInfo());

	/*
	 * Update pose. Note that coordinate system of the human model is different to ROS REP 105
	 * https://www.ros.org/reps/rep-0105.html
	 */
	std::cout << "LOADED POSE: " << actor_ptr_->WorldPose() << std::endl;
	sim_localisation_->update(actor_ptr_->WorldPose());
	actor_ptr_->SetWorldPose(sim_localisation_->getPoseTransformed());
}

void ActorPlugin::Reset() {

}

void ActorPlugin::initializeHuberoInterface() {
	// animation control
	sim_animation_control_->initialize(
		actor_ptr_->SkeletonAnimations(),
		// TODO: param
		hubero::ANIMATION_STAND
	);

	// localisation
	// TODO: param
	sim_localisation_->initialize("world");

	// model control
	// TODO: param
	sim_model_control_->initialize(actor_ptr_, "world");

	// world geometry
	// TODO: param
	sim_world_geometry_->initialize("world", world_ptr_, actor_ptr_->GetName());
}

void ActorPlugin::OnUpdate(const common::UpdateInfo& info) {
	/*
	 * This is very naive but the most effective way to prepare both ROS and Gazebo for typical operation.
	 */
	if (!controller_enabled_) {
		// returns seconds since plugin load
		if (info.realTime.Double() >= 8.0 ) {
			std::cout << "\t[ActorPlugin] Actor controller starting the job!" << std::endl;
			controller_enabled_ = true;
		}
		return;
	}

	/*
	 * Handle simulation update
	 */
	sim_localisation_->update(actor_ptr_->WorldPose());
	hubero::Time time(info.simTime.Double());
	hubero_actor_.update(time);
}

} // namespace gazebo
