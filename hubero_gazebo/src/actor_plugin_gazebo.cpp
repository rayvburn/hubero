#include <hubero_gazebo/actor_plugin_gazebo.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::ActorPlugin)

namespace gazebo {

ActorPlugin::ActorPlugin():
	controller_enabled_(false),
	sim_animation_control_ptr_(std::make_shared<hubero::AnimationControlGazebo>()),
	sim_localisation_ptr_(std::make_shared<hubero::LocalisationGazebo>()),
	sim_model_control_ptr_(std::make_shared<hubero::ModelControlGazebo>()),
	sim_world_geometry_ptr_(std::make_shared<hubero::WorldGeometryGazebo>()),
	ros_node_ptr_(std::make_shared<hubero::Node>("hubero_gazebo_ros_node")),
	ros_task_ptr_(std::make_shared<hubero::TaskRequestRos>()),
	ros_nav_ptr_(std::make_shared<hubero::NavigationRos>())
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
	 * HuBeRo framework simulator interfaces initialization
	 */
	// TODO: consider parameterization of the initial animation type
	sim_animation_control_ptr_->initialize(
		std::bind(&gazebo::physics::Actor::SetCustomTrajectory, actor_ptr_, std::placeholders::_1),
		actor_ptr_->SkeletonAnimations(),
		hubero::AnimationType::ANIMATION_STAND
	);
	sim_localisation_ptr_->initialize(ros_node_ptr_->getSimulatorFrame());
	sim_model_control_ptr_->initialize(actor_ptr_, ros_node_ptr_->getSimulatorFrame());
	sim_world_geometry_ptr_->initialize(ros_node_ptr_->getSimulatorFrame(), world_ptr_, actor_ptr_->GetName());

	/*
	 * Update pose. Note that coordinate system of the human model is different to ROS REP 105
	 * https://www.ros.org/reps/rep-0105.html
	 */
	sim_localisation_ptr_->updateSimulator(actor_ptr_->WorldPose());
	actor_ptr_->SetWorldPose(sim_localisation_ptr_->getPoseSimulator());

	/*
	 * HuBeRo framework task interface initialization
	 */
	ros_task_ptr_->initialize(ros_node_ptr_, actor_ptr_->GetName(), ros_node_ptr_->getSimulatorFrame());

	/*
	 * HuBeRo framework navigation interface initialization
	 */
	ros_nav_ptr_->initialize(ros_node_ptr_,
		actor_ptr_->GetName(),
		ros_node_ptr_->getSimulatorFrame(),
		sim_localisation_ptr_->getPose()
	);

	/*
	 * Initialize HuBeRo - provide interface classes
	 */
	hubero_actor_.initialize(
		actor_ptr_->GetName(),
		sim_animation_control_ptr_,
		sim_model_control_ptr_,
		sim_world_geometry_ptr_,
		sim_localisation_ptr_,
		ros_nav_ptr_,
		ros_task_ptr_
	);

	/*
	 * Enable specific trajectory of actor
	 * It must be done in "Load", otherwise default animation (running) will be triggered and all actors in the 'world'
	 * will be located in exact the same place
	 */
	actor_ptr_->SetCustomTrajectory(sim_animation_control_ptr_->getTrajectoryInfo());
}

void ActorPlugin::Reset() {

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
	sim_localisation_ptr_->updateSimulator(actor_ptr_->WorldPose());
	hubero::Time time(info.simTime.Double());
	hubero_actor_.update(time);
}

} // namespace gazebo
