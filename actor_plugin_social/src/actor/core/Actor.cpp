/*
 * Actor.cpp
 *
 *  Created on: Apr 9, 2019
 *      Author: rayvburn
 */

#include <actor/core/Actor.h>
#include <algorithm>    		// std::find
#include <ignition/math/Line3.hh>
#include <memory> 				// make_unique
#include <std_msgs/Float32.h> 	// ActorObstacleMsgType

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

Actor::Actor():
 		bounding_type_(ACTOR_BOUNDING_ELLIPSE),
		bounding_ptr_(nullptr),
		stance_(ACTOR_STANCE_STAND),
		trans_function_ptr(nullptr)
{}

// ------------------------------------------------------------------- //

void Actor::initGazeboInterface(const gazebo::physics::ActorPtr &actor, const gazebo::physics::WorldPtr &world) {

	// copy Gazebo world data pointers
	actor_ptr_ = actor;
	world_ptr_ = world;

	/* add an instance to common info class (missing velocity,
	 * bounding box of Actor in WorldPtr) */
	common_info_ptr_ = std::make_shared<actor::core::CommonInfo>();
	common_info_ptr_->addActor(actor_ptr_->GetName());

}

// ------------------------------------------------------------------- //

void Actor::initRosInterface() {

	// run parameter loader
	params_ptr_ = std::make_shared<actor::ros_interface::ParamLoader>();
	params_ptr_->setActorParamsPrefix("actor");	// if this prefix will be changed then "actor_global_plan_node"'s SetPlannerTolerance function must be adjusted accordingly
	params_ptr_->setSfmParamsPrefix("sfm");
	params_ptr_->setBehaviourParamsPrefix("beh");
	params_ptr_->loadParameters(node_.getNodeHandlePtr());

	// assign the parameter value to the `actor_global_frame_id`;
	// the parameter is also used by global plan ROS node
	frame_global_.setFrame(params_ptr_->getActorParams().global_frame_name);

	/* initialize SFM visualization instances */
	sfm_vis_arrow_.init(frame_global_.getFrame());
	sfm_vis_arrow_.setParameters(1.0f, params_ptr_->getSfmParams().max_force);

	sfm_vis_text_.init(frame_global_.getFrame());
	sfm_vis_text_.setColor(0.9f, 0.9f, 0.9f, 0.95f);
	sfm_vis_text_.setParameters(0.2f);

	sfm_vis_line_list_.init(frame_global_.getFrame());
	sfm_vis_line_list_.setColor(1.0f, 1.0f, 0.0f, 0.7f);

	if ( params_ptr_->getSfmVisParams().publish_grid ) {
		// `sfm_vis_grid_` initializes a new arrow instance (must not be confused with
		// the `sfm_vis_arrow_`) due to the inheritance
		sfm_vis_grid_.init(frame_global_.getFrame());
		sfm_vis_grid_.setParameters(params_ptr_->getSfmVisParams().grid_resolution, params_ptr_->getSfmParams().max_force);
		sfm_vis_grid_.setColor(0.2f, 1.0f, 0.0f, 0.7f);
		/* make grid artificially bigger than a world's bounds;
		 * it makes random targets to be located further from
		 * walls */
		sfm_vis_grid_.createGrid(params_ptr_->getActorParams().world_bound_x.at(0) - 3.0, params_ptr_->getActorParams().world_bound_x.at(1) + 3.0,
								 params_ptr_->getActorParams().world_bound_y.at(0) - 3.0, params_ptr_->getActorParams().world_bound_y.at(1) + 3.0,
								 params_ptr_->getSfmVisParams().grid_resolution);
	}

	if ( params_ptr_->getSfmVisParams().publish_potential ) {
		sfm_vis_heatmap_.init(frame_global_.getFrame());
		sfm_vis_heatmap_.setParameters(params_ptr_->getSfmParams().min_force,
									   params_ptr_->getSfmParams().max_force,
									   params_ptr_->getSfmVisParams().potential_resolution);
		sfm_vis_heatmap_.createGrid(params_ptr_->getActorParams().world_bound_x.at(0) - 3.0, params_ptr_->getActorParams().world_bound_x.at(1) + 3.0,
								 	params_ptr_->getActorParams().world_bound_y.at(0) - 3.0, params_ptr_->getActorParams().world_bound_y.at(1) + 3.0,
									params_ptr_->getSfmVisParams().potential_resolution);

	}


	/* initialize ROS interface to allow publishing and receiving messages;
	 * due to inheritance from `enable_shared_from_this`, this method is created
	 * as a separate one */
	stream_.setNodeHandle(node_.getNodeHandlePtr());
	stream_.setNamespace(actor_ptr_->GetName());
	stream_.initPublisher<ActorMarkerType, visualization_msgs::Marker>(ActorMarkerType::ACTOR_MARKER_BOUNDING, "bounding");
	stream_.initPublisher<ActorMarkerType, visualization_msgs::Marker>(ActorMarkerType::ACTOR_MARKER_INTERNAL_VECTOR, "sfm_internal");
	stream_.initPublisher<ActorMarkerType, visualization_msgs::Marker>(ActorMarkerType::ACTOR_MARKER_INTERACTION_VECTOR, "sfm_interaction");
	stream_.initPublisher<ActorMarkerType, visualization_msgs::Marker>(ActorMarkerType::ACTOR_MARKER_SOCIAL_VECTOR, "sfm_social");
	stream_.initPublisher<ActorMarkerType, visualization_msgs::Marker>(ActorMarkerType::ACTOR_MARKER_COMBINED_VECTOR, "sfm_combined");
	stream_.initPublisher<ActorMarkerTextType, visualization_msgs::Marker>(ActorMarkerTextType::ACTOR_MARKER_TEXT_BEH, "behaviour");
	stream_.initPublisher<ActorMarkerArrayType, visualization_msgs::MarkerArray>(ActorMarkerArrayType::ACTOR_MARKER_ARRAY_CLOSEST_POINTS, "closest_points");
	stream_.initPublisher<ActorNavMsgType, nav_msgs::Path>(ActorNavMsgType::ACTOR_NAV_PATH_GLOBAL, "path");
	stream_.initPublisher<ActorNavMsgType, nav_msgs::Path>(ActorNavMsgType::ACTOR_NAV_PATH_REAL, "path_real");
	stream_.initPublisher<ActorObstacleMsgType, std_msgs::Float32>(ActorObstacleMsgType::ACTOR_OBSTACLE_DIST_CLOSEST, "dist_obstacle");

	// check if grid usage has been enabled in .YAML file
	if ( params_ptr_->getSfmVisParams().publish_grid ) {
		stream_.initPublisher<ActorMarkerArrayType, visualization_msgs::MarkerArray>(ActorMarkerArrayType::ACTOR_MARKER_ARRAY_GRID, "force_grid");
	}

	// check if potential field usage has been enabled in .YAML file
	if ( params_ptr_->getSfmVisParams().publish_potential ) {
		stream_.initPublisher<ActorMarkerArrayType, visualization_msgs::MarkerArray>(ActorMarkerArrayType::ACTOR_MARKER_ARRAY_POTENTIAL, "potential_field");
	}

	// constructor of a Connection object
	connection_ptr_ = std::make_shared<actor::ros_interface::Connection>();

	// Connection's configuration
	connection_ptr_->setActorPtr( shared_from_this() ); // passes `std::shared_ptr` of `this` (which normally would be a const pointer)
	connection_ptr_->setNodeHandle(node_.getNodeHandlePtr());
	connection_ptr_->setNamespace(actor_ptr_->GetName());

	// initialize services for Actor control
	connection_ptr_->initServices();
	connection_ptr_->initActions();
	connection_ptr_->startCallbackProcessingThread();

}

// ------------------------------------------------------------------- //

void Actor::initInflator(const double &circle_radius) {

	// got rid of the raw pointer, ref: https://stackoverflow.com/questions/4665266/creating-shared-ptr-from-raw-pointer
	// circle
	std::shared_ptr<actor::inflation::Circle> bounding_circle_ptr = std::make_shared<actor::inflation::Circle>();
	bounding_type_ = ACTOR_BOUNDING_CIRCLE;
	bounding_circle_ptr->setRadius(circle_radius);
	bounding_circle_ptr->updatePose(*pose_world_ptr_);
	bounding_ptr_ = bounding_circle_ptr;
	common_info_ptr_->setBorderPtr(bounding_ptr_);

}

// ------------------------------------------------------------------- //

void Actor::initInflator(const double &box_x_half, const double &box_y_half, const double &box_z_half) {

	// box
	std::shared_ptr<actor::inflation::Box> bounding_box_ptr = std::make_shared<actor::inflation::Box>();
	bounding_type_ = ACTOR_BOUNDING_BOX;
	bounding_box_ptr->init(box_x_half, box_y_half, box_z_half);
	bounding_box_ptr->updatePose(*pose_world_ptr_);
	bounding_ptr_ = bounding_box_ptr;
	common_info_ptr_->setBorderPtr(bounding_ptr_);

}

// ------------------------------------------------------------------- //

void Actor::initInflator(const double &semi_major, const double &semi_minor, const double &center_offset_x,
						 const double &center_offset_y) {

	// ellipse
	std::shared_ptr<actor::inflation::Ellipse> bounding_ellipse_ptr = std::make_shared<actor::inflation::Ellipse>();
	bounding_type_ = ACTOR_BOUNDING_ELLIPSE;

	// correct yaw angle to make ellipse abstract from Actor coordinate system's orientation
	ignition::math::Angle yaw_world( pose_world_ptr_->Rot().Yaw() - IGN_PI_2);
	yaw_world.Normalize();
	bounding_ellipse_ptr->init( semi_major, semi_minor, yaw_world.Radian(), pose_world_ptr_->Pos(), ignition::math::Vector3d(center_offset_x, center_offset_y, 0.0) );
	bounding_ptr_ = bounding_ellipse_ptr;
	common_info_ptr_->setBorderPtr(bounding_ptr_);

}

// ------------------------------------------------------------------- //

void Actor::initSFM() {

	/* choose a proper inflation type - convert from actor's inflation types
	 * available; number of types are not equal for actor and for SFM thus
	 * a conversion is done */
	sfm::InflationType sfm_inflation;

	/* using param value instead of member variable
	 * in case of different module initialization order */
	switch ( static_cast<actor::ActorBoundingType>(params_ptr_->getActorInflatorParams().bounding_type)  ) {

		case(actor::ActorBoundingType::ACTOR_BOUNDING_BOX):

			/* based on `box inflation type` parameter - choose the right
			 * SFM's box inflation type (actor is insensitive to this parameter);
			 * see sfm::InflationType */
			if ( static_cast<sfm::InflationType>(params_ptr_->getSfmParams().box_inflation_type) ==
				 sfm::InflationType::INFLATION_BOX_ALL_OBJECTS ) {
				sfm_inflation = sfm::InflationType::INFLATION_BOX_ALL_OBJECTS;

			} else if ( static_cast<sfm::InflationType>(params_ptr_->getSfmParams().box_inflation_type) ==
						sfm::InflationType::INFLATION_BOX_OTHER_OBJECTS ) {
				sfm_inflation = sfm::InflationType::INFLATION_BOX_OTHER_OBJECTS;
			}
			break;

		case(actor::ActorBoundingType::ACTOR_BOUNDING_CIRCLE):
				sfm_inflation = sfm::InflationType::INFLATION_CIRCLE;
				break;

		case(actor::ActorBoundingType::ACTOR_BOUNDING_ELLIPSE):
				sfm_inflation = sfm::InflationType::INFLATION_ELLIPSE;
				break;

		case(actor::ActorBoundingType::ACTOR_NO_BOUNDING):
				/* this bounding type seems to not have a correspondence in code? */
				sfm_inflation = sfm::InflationType::INFLATION_NONE;
				break;

	}

	// initialize SFM parameters
	sfm_.init(params_ptr_, sfm_inflation, actor_ptr_->GetName(), world_ptr_);
	
}

// ------------------------------------------------------------------- //

void Actor::initActor(const sdf::ElementPtr sdf) {

	// - - - - - - - - - - - - - - - - - - - - - - -
	// finite state machine setup section
	// add states for FSM
	fsm_.addState(ACTOR_STATE_ALIGN_TARGET);
	fsm_.addState(ACTOR_STATE_MOVE_AROUND);
	fsm_.addState(ACTOR_STATE_TARGET_REACHING);
	fsm_.addState(ACTOR_STATE_LIE_DOWN);
	fsm_.addState(ACTOR_STATE_STOP_AND_STARE);
	fsm_.addState(ACTOR_STATE_FOLLOW_OBJECT);
	fsm_.addState(ACTOR_STATE_TELEOPERATION);

	// update state handler function pointer
	if ( fsm_.didStateChange() ) {
		updateTransitionFunctionPtr();
	}

	// - - - - - - - - - - - - - - - - - - - - - - -
	// target manager initialization section
	pose_world_ptr_ = std::make_shared<ignition::math::Pose3d>();
	target_manager_ = Target(world_ptr_, pose_world_ptr_, params_ptr_);
	target_manager_.initializeGlobalPlan(node_.getNodeHandlePtr(), 40, actor_ptr_->GetName());
	target_manager_.initializeCommonInfo(common_info_ptr_);


	// - - - - - - - - - - - - - - - - - - - - - - -
	// initial stance setup section
	// set initial stance and create custom trajectory
	setStance( static_cast<actor::ActorStance>(params_ptr_->getActorParams().init_stance) );


	// - - - - - - - - - - - - - - - - - - - - - - -
	// initial pose setup section
	/* set starting pose different to default to prevent actor
	 * lying on his back 1 m above the ground */
	ignition::math::Pose3d init_pose;

	// check if some values have been set in .YAML
	if ( params_ptr_->getActorParams().init_pose.size() == 0 ) {

		// process parameters added to .world file (.sdf)
		ignition::math::Vector3d init_orient = actor_ptr_->WorldPose().Rot().Euler();
		init_pose.Set( actor_ptr_->WorldPose().Pos(), ignition::math::Quaterniond(init_orient.X(), init_orient.Y(), init_orient.Z()) );

	} else {

		// set pose as set in .YAML file - handy for single actor
		// not very handy for 100 of them
		init_pose.Set( params_ptr_->getActorParams().init_pose.at(0), params_ptr_->getActorParams().init_pose.at(1), params_ptr_->getActorParams().init_pose.at(2), params_ptr_->getActorParams().init_pose.at(3), params_ptr_->getActorParams().init_pose.at(4), params_ptr_->getActorParams().init_pose.at(5) );

	}
	updateStanceOrientation(init_pose);
	actor_ptr_->SetWorldPose(init_pose);

	// set previous pose to prevent velocity overshoot
	pose_world_prev_ptr_ = std::make_shared<ignition::math::Pose3d>();
	*pose_world_prev_ptr_ = actor_ptr_->WorldPose();

	// set initial pose (reference point for global plan generation)
	*pose_world_ptr_ = actor_ptr_->WorldPose();

	// - - - - - - - - - - - - - - - - - - - - - - -
	// velocity setup
	velocity_.configure(pose_world_ptr_, pose_world_prev_ptr_);

	// - - - - - - - - - - - - - - - - - - - - - - -
	// bounding setup section
	bounding_ptr_ = std::make_shared<actor::inflation::Border>();

	// convert int to enum value and initialize a proper inflator/bounding
	bounding_type_ = static_cast<actor::ActorBoundingType>(params_ptr_->getActorInflatorParams().bounding_type);

	switch ( bounding_type_ ) {
	case(ACTOR_BOUNDING_BOX):
			initInflator( params_ptr_->getActorInflatorParams().box_size.at(0), params_ptr_->getActorInflatorParams().box_size.at(1), params_ptr_->getActorInflatorParams().box_size.at(2) );
			break;
	case(ACTOR_BOUNDING_CIRCLE):
			initInflator( params_ptr_->getActorInflatorParams().circle_radius );
			break;
	case(ACTOR_BOUNDING_ELLIPSE):
			initInflator( params_ptr_->getActorInflatorParams().ellipse.at(0), params_ptr_->getActorInflatorParams().ellipse.at(1), params_ptr_->getActorInflatorParams().ellipse.at(2), params_ptr_->getActorInflatorParams().ellipse.at(3) );
			break;
	}


	// - - - - - - - - - - - - - - - - - - - - - - -
	// initial target setup section

	// check if target coordinates have been set in .YAML - vector of 3 elements expected
	if ( params_ptr_->getActorParams().init_target.size() == 3 ) {

		// set target according to .YAML
		target_manager_.setNewTarget(ignition::math::Vector3d( params_ptr_->getActorParams().init_target.at(0),
															   params_ptr_->getActorParams().init_target.at(1),
															   params_ptr_->getActorParams().init_target.at(2) ));

	} else if ( sdf && sdf->HasElement("target") ) {

		// target coordinates in .YAML haven't been defined - use .sdf
		sdf::ElementPtr target_elem = sdf->GetElement("target")->GetElement("point");
		while (target_elem) {
			// readability
			ignition::math::Vector3d pt = target_elem->Get<ignition::math::Vector3d>();
			// add straight to the queue
			target_manager_.setNewTarget(pt, true);
			// try to get next element of the list
			target_elem = target_elem->GetNextElement("point");
		}

	} else {

		// No target set but do not try to choose it now as costmap is very likely
		// not initialized yet. Proper state handlers will take care of that afterwards.

	}

	// - - - - - - - - - - - - - - - - - - - - - - -
	// ignored models section

	// check if model names have been set in .YAML (YAML takes precedence over the .world file configuration)
	if ( params_ptr_->getSfmDictionary().ignored_models_.size() == 0 ) {

		// ignored models haven't been defined in .YAML
		if ( sdf && sdf->HasElement("ignore_obstacles") ) {

			// // evaluate .world file contents - use .sdf
			sdf::ElementPtr model_elem = sdf->GetElement("ignore_obstacles")->GetElement("model");
			while (model_elem) {
				// readability
				std::string model = model_elem->Get<std::string>();
				// extend dictionary
				params_ptr_->getSfmDictionary().ignored_models_.push_back(model);
				// try to get next element of the list
				model_elem = model_elem->GetNextElement("model");
			}

		}

	}

	// - - - - - - - - - - - - - - - - - - - - - - -
	// initialize SFM with loaded set of parameters
	initSFM();

	// - - - - - - - - - - - - - - - - - - - - - - -
	// initialize SocialConductor with loaded set of parameters
	social_conductor_.configure(params_ptr_->getBehaviourParams());

}

// ------------------------------------------------------------------- //

bool Actor::moveAround() {

	// FIXME: evaluate the current state correctly
	if ( fsm_.getState() == ActorState::ACTOR_STATE_MOVE_AROUND ||
		 fsm_.getState() == ActorState::ACTOR_STATE_ALIGN_TARGET ) // valid just after switching to move around
	{
		return (false);
	} else if ( fsm_.getState() == ActorState::ACTOR_STATE_TARGET_REACHING ) {
		return (false);
	}

	// NOTE: at the moment switching directly from
	// the "FollowObject" state to the "MoveAround"
	// state is not allowed -
	// an explicit "stop_following" service call
	// must be executed
	if ( target_manager_.isFollowing() ) {
		// NOTE: uncommenting allows direct switch (described above)
		//target_manager_.stopFollowing();
		return (false);
	}

	// reset the current target
	if ( target_manager_.isTargetChosen() ) {
		target_manager_.abandonTarget();
	}

	// clear the target queue
	while ( !target_manager_.isTargetQueueEmpty() ) {
		target_manager_.setNewTarget(); 	// pull out a target from the queue
		target_manager_.abandonTarget();	// abandon it
	}

	action_info_.start();
	action_info_.setStatus(Action::MovingAroundStatus::ROTATE_TOWARDS_OBJECT, "rotation towards object direction");

	// NOTE: to make actor face the target first,
	// there must be an order of calls as below:
	setState(ActorState::ACTOR_STATE_MOVE_AROUND);
	setState(ActorState::ACTOR_STATE_ALIGN_TARGET);
	// this forces FSM to back to follow object state after
	// successful alignment (previous state is equal to
	// `follow_object` after alignment)
	//
	// setStance after setState because setState updates default
	// stance for a given state
	setStance(ActorStance::ACTOR_STANCE_WALK);

	return (true);
}

// ------------------------------------------------------------------- //

bool Actor::moveAroundStop() {

	// evaluate if the Actor actually operates in "MoveAround" state
	if ( fsm_.getState() != ActorState::ACTOR_STATE_MOVE_AROUND &&
		 fsm_.getState() != ActorState::ACTOR_STATE_ALIGN_TARGET ) // valid just after switching to move around
	{
		return (false);
	}

	// reset the current target
	if ( target_manager_.isTargetChosen() ) {
		target_manager_.abandonTarget();
	}

	action_info_.setStatus(Action::FINISHED, "finished operation");

	// update the FSM state and the stance
	setState(ActorState::ACTOR_STATE_STOP_AND_STARE);
	setStance(ActorStance::ACTOR_STANCE_STAND);

	resetVisualization();

	return (true);
}

// ------------------------------------------------------------------- //

bool Actor::followObject(const std::string &object_name) {

	if ( target_manager_.followObject(object_name) ) {

		action_info_.start();
		action_info_.setStatus(Action::FollowObjectStatus::ROTATE_TOWARDS_OBJECT, "rotation towards object direction");

		follow_object_.start();

		ignored_models_.push_back(object_name);
		// NOTE: to make actor face the target first,
		// there must be an order of calls as below:
		setState(ActorState::ACTOR_STATE_FOLLOW_OBJECT);
		setState(ActorState::ACTOR_STATE_ALIGN_TARGET);
		// this forces FSM to back to follow object state after
		// successful alignment (previous state is equal to
		// `follow_object` after alignment)
		//
		// setStance after setState because setState updates default
		// stance for a given state
		setStance(ActorStance::ACTOR_STANCE_WALK);
		return (true);

	}
	return (false);

}

// ------------------------------------------------------------------- //

bool Actor::followObjectStop() {

	if ( !target_manager_.isFollowing() ) {
		return (false);
	}

	target_manager_.abandonTarget();
	target_manager_.stopFollowing();

	if ( ignored_models_.size() > 0 ) {
		ignored_models_.pop_back(); // FIXME: it won't work if ignored_models stores other elements than followed object's name
	}

	setState(ActorState::ACTOR_STATE_STOP_AND_STARE);

	action_info_.setStatus(Action::FINISHED, "finished operation");
	resetVisualization();

	return (true);

}

// ------------------------------------------------------------------- //

bool Actor::lieDown(const std::string &object_name, const double &height, const double &rotation) {

	if ( lie_down_.isLyingDown() ) {
		std::cout << "Actor::lieDown - stop lying first, actor can't grovel" << std::endl;
		return (false);
	}

	lie_down_.setLying(false);
	lie_down_.setLyingHeight(height);
	lie_down_.setRotation(rotation);

	// `isModelValid` - in fact this operation is executed 2 times..
	bool valid = false;
	gazebo::physics::ModelPtr model_p = nullptr;
	std::tie(valid, model_p) = target_manager_.isModelValid(object_name);
	if ( !valid ) {
		return (false);
	}
	lie_down_.setLyingPose(model_p->WorldPose());

	// NOTE: clears the current queue - hard-coded (`false` below)
	bool status = target_manager_.setNewTargetPriority(object_name, false);

	// if new-desired target is achievable then change the state (align firstly)
	if ( status ) {

		action_info_.start();
		action_info_.setStatus(Action::LieDownStatus::ROTATE_TOWARDS_OBJECT, "rotation");

		setState(ActorState::ACTOR_STATE_LIE_DOWN); // will be restored by FSM
		setState(ActorState::ACTOR_STATE_ALIGN_TARGET);
		setStance(ActorStance::ACTOR_STANCE_WALK);

	}

	return (status);

}

// ------------------------------------------------------------------- //

bool Actor::lieDown(const double &x_pos, const double &y_pos, const double &z_pos, const double &rotation) {

	if ( lie_down_.isLyingDown() ) {
		std::cout << "Actor::lieDown - stop lying first, actor can't grovel" << std::endl;
		return (false);
	}

	lie_down_.setLying(false);
	lie_down_.setLyingHeight(z_pos);
	lie_down_.setRotation(rotation);
	lie_down_.setLyingPose(ignition::math::Pose3d(x_pos, y_pos, z_pos,
												  IGN_PI_2, 0.0, 0.0)); // orientation: hard-coded STAND/WALK configuration

	// NOTE: clears the current queue - hard-coded (`false` below)
	bool status = target_manager_.setNewTargetPriority(ignition::math::Vector3d(x_pos, y_pos, 0.0), false);

	// if new-desired target is achievable then change the state (align firstly)
	if ( status ) {

		action_info_.start();
		action_info_.setStatus(Action::LieDownStatus::ROTATE_TOWARDS_OBJECT, "rotation");

		setState(ActorState::ACTOR_STATE_LIE_DOWN); // will be restored by FSM
		setState(ActorState::ACTOR_STATE_ALIGN_TARGET);
		setStance(ActorStance::ACTOR_STANCE_WALK);

	}

	return (status);

}

// ------------------------------------------------------------------- //

bool Actor::lieDownStop() {

	if ( !lie_down_.isLyingDown() ) {
		return (false);
	}
	lie_down_.stopLying();
	action_info_.setStatus(Action::FINISHED, "finished operation");
	// no need to reset internal state (configuration variables: poses etc)
	return (true);

}

// ------------------------------------------------------------------- //

std::array<double, 3> Actor::getVelocity() const {

	std::array<double, 3> array;
	array.at(0) = velocity_.getLinear().X();
	array.at(1) = velocity_.getLinear().Y();
	array.at(2) = velocity_.getLinear().Z();
	return (array);

}

// ------------------------------------------------------------------- //

std::array<double, 6> Actor::getPose() const {

	std::array<double, 6> array;
	array.at(0) = pose_world_ptr_->Pos().X();
	array.at(1) = pose_world_ptr_->Pos().Y();
	array.at(2) = pose_world_ptr_->Pos().Z();
	array.at(3) = pose_world_ptr_->Rot().Roll();
	array.at(4) = pose_world_ptr_->Rot().Pitch();

	// transform actor orientation to world coordinate system
	ignition::math::Angle yaw_actor_w(pose_world_ptr_->Rot().Yaw() - IGN_PI_2);
	yaw_actor_w.Normalize();
	array.at(5) = yaw_actor_w.Radian();

	return (array);

}

// ------------------------------------------------------------------- //

const Action Actor::getActionInfo() const {
	return (action_info_);
}

// ------------------------------------------------------------------- //

bool Actor::setStance(const actor::ActorStance &stance_type) {

	if ( stance_ != stance_type ) {

		stance_ = stance_type;
		std::string animation = convertStanceToAnimationName();
		gazebo::physics::Actor::SkeletonAnimation_M skeleton_anims = actor_ptr_->SkeletonAnimations();

		/* To print available animations:
		for (auto& x: skeleton_anims) { std::cout << "Skel. animation: " << x.first << std::endl; }
		*/

		if ( skeleton_anims.find(animation) == skeleton_anims.end() ) {

			// print warning
			std::cout << "[STANCE] Skeleton animation " << animation << " not found.\n";
			return (false);

		} else {

			// Create custom trajectory
			trajectory_info_.reset(new gazebo::physics::TrajectoryInfo());
			trajectory_info_->type = animation;
			trajectory_info_->duration = 1.0;
			actor_ptr_->SetCustomTrajectory(trajectory_info_);

			// debug info
			std::cout << "[STANCE] " << actor_ptr_->GetName() << "'s\tnew stance:\t";
			if ( stance_ != ACTOR_STANCE_LIE ) {
				std::cout << animation << std::endl;
			} else {
				// `lie` must be handled differently as the `lie` is equal to `stand`
				// but in a different plane
				std::cout << "lie" << std::endl;
			}

			return (true);

		}

	}
	return (false);

}

// ------------------------------------------------------------------- //

bool Actor::setState(const actor::ActorState &new_state) {

	// check if state is valid
	unsigned int state_to_be = static_cast<unsigned int>(new_state);
	unsigned int state_lower_bound = static_cast<unsigned int>(actor::ACTOR_STATE_ALIGN_TARGET);

	// NOTE: below needs to be adjusted if new state will be added!
	// same note added in the `Enum.h` file
	unsigned int state_upper_bound = static_cast<unsigned int>(actor::ACTOR_STATE_TELEOPERATION);

	if ( (state_to_be >= state_lower_bound) && (state_to_be <= state_upper_bound) ) {
		fsm_.setState(new_state);
		updateTransitionFunctionPtr();
		return (true);
	} else {
		return (false);
	}

}

// ------------------------------------------------------------------- //

void Actor::executeTransitionFunction() {
	(this->*trans_function_ptr)();
}

// ------------------------------------------------------------------- //

void Actor::stateHandlerAlignTarget() {

	// ---------------------------------------------

	prepareForUpdate();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// copy the actor's current rotation to local variable
	ignition::math::Vector3d new_rpy = pose_world_ptr_->Rot().Euler();

	/* if already aligned - switch to a certain state, otherwise proceed to the next
	 * rotation procedure */
	if ( alignToTargetDirection(&new_rpy) ) {
#ifndef SILENT_
		std::cout << "\n\n\t\t\tALIGNED\n\n";
#endif
		// change FSM state if aligned
		if ( fsm_.getStatePrevious() != ACTOR_STATE_ALIGN_TARGET ) {
			// restore previous only if the previous one is not the current one,
			// otherwise actor will be stuck
			setState(fsm_.getStatePrevious());
		} else {
			// select experimentally
			setState(actor::ACTOR_STATE_MOVE_AROUND);
		}
		resetVisualization(false);

	}

	// update the local copy of the actor's pose (orientation only)
	ignition::math::Quaterniond quat(new_rpy);
	pose_world_ptr_->Rot() = quat;

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	/* forced close-to-zero distance traveled to avoid actor oscillations;
	 * of course 0.0 linear distance is traveled when pure rotation is performed */
	applyUpdate(params_ptr_->getActorParams().animation_speed_rotation);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerMoveAround() {

	// ---------------------------------------------

	// Social Force Model
	double dt = prepareForUpdate();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// check whether target is still available and state does not need to be changed
	if ( !manageTargetMovingAround() ) {
		return;
	}

	double dist_traveled = move(dt);
	visualizeSfmCalculations();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	applyUpdate(dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerTargetReaching() {

	double dt = prepareForUpdate();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	if ( !manageTargetSingleReachment() ) {
		target_manager_.abandonTarget();
		setState(actor::ACTOR_STATE_STOP_AND_STARE);
		resetVisualization();
		// state termination request moved away from
		// the `manageTargetSingleReachment`
		// to allow reuse of the PTF in other
		// transition functions
		action_info_.terminate();
		return;
	}

	double dist_traveled = move(dt);
	visualizeSfmCalculations();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	applyUpdate(dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerLieDown() {

	double dt = prepareForUpdate();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	double dist_traveled = 0.001; // default (determines speed of animation)

	// move until a calculated position is reached,
	// i.e. no new target is chosen
	if ( target_manager_.isTargetChosen() ) {

		if ( !manageTargetSingleReachment() ) {
			// executes only once
			target_manager_.abandonTarget();
		}
		dist_traveled = move(dt);
		visualizeSfmCalculations();

	} else if ( !lie_down_.isLyingDown() ) {

		action_info_.setStatus(Action::LieDownStatus::LYING, "lying");

		// actor arrived to the target point;
		// let's lie down
		setStance(ActorStance::ACTOR_STANCE_LIE);

		// update state's internal configuration
		lie_down_.setLying(true);

		// save configuration
		lie_down_.setPoseBeforeLying(*pose_world_ptr_); // will be restored
		*pose_world_ptr_ = lie_down_.getPoseLying();	// move to the desired point

		resetVisualization();
		// now, wait for interruption

	} else if ( lie_down_.doStopLying() ){

		action_info_.setStatus(Action::LieDownStatus::STANDING_UP, "standing up");

		// reset internal state
		lie_down_.setLying(false);

		// process stop lying call
		*pose_world_ptr_ = lie_down_.computePoseFinishedLying();
		*pose_world_prev_ptr_ = *pose_world_ptr_;

		// no need to reset SFM here,
		// `setStance` is not needed either because STAND stance
		// is active during lying and after stand up also (different
		// orientation)
		setState(ActorState::ACTOR_STATE_STOP_AND_STARE);
		action_info_.setStatus(Action::FINISHED, "finished operation");

	} else {

		// LieDown - idle
		visualizeSfmCalculations();
//		std::cout << "LIE DOWN IDLE" << std::endl;

	}


	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	applyUpdate(dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerStopAndStare() {

	double dt = prepareForUpdate();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// stopped, does nothing
	// blank handler
	visualizeSfmCalculations();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	double dist_traveled = 0.001;
	applyUpdate(dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerFollowObject() {

	double dt = prepareForUpdate();
	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// update position of the tracked object and generate global plan every few seconds
	if ( !manageTargetTracking() ) {
		return;
	}

	double dist_traveled = move(dt);
	visualizeSfmCalculations();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	applyUpdate(dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::stateHandlerTeleoperation() {

	double dt = prepareForUpdate();

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	// this handler will be filled if needed

	// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

	double dist_traveled = 0.001; // default animation speed
	applyUpdate(dist_traveled);

}

// ------------------------------------------------------------------- //



// ------------------------------------------------------------------- //
// private section
// ------------------------------------------------------------------- //

void Actor::updateStanceOrientation(ignition::math::Pose3d &pose) {

	/* Corrects the rotation to align face with x axis if yaw = 0
	 * and stand up (with roll 0 actor is lying) */

	ignition::math::Vector3d rpy = pose.Rot().Euler();

	switch (stance_) {

		case(actor::ACTOR_STANCE_WALK):
		case(actor::ACTOR_STANCE_STAND):
		case(actor::ACTOR_STANCE_STAND_UP):
		case(actor::ACTOR_STANCE_TALK_A):
		case(actor::ACTOR_STANCE_TALK_B):
		case(actor::ACTOR_STANCE_RUN):
		case(actor::ACTOR_STANCE_SIT_DOWN):
		case(actor::ACTOR_STANCE_SITTING):
				rpy.X(IGN_PI_2);
				break;

		case(actor::ACTOR_STANCE_LIE):
				rpy.X(0.0000);
				break;

	}

	pose.Rot().Euler(rpy);

}

// ------------------------------------------------------------------- //

/// \brief Calculate actor's rotation angle (`YAW`) that needs to be applied to
/// make actor face the target direction. Rotation is performed around world's Z axis.
/// \note 90 degrees rotation applied because actor coordinate system is rotated 90 degrees CW.
bool Actor::alignToTargetDirection(ignition::math::Vector3d *rpy) {

	// NOTE: Calculations considered in the world coordinate system

	// calculate `to_target_vector` which connects actor current position with current checkpoint
	ignition::math::Vector3d to_target_vector = target_manager_.getCheckpoint() - pose_world_ptr_->Pos();

	// calculate `to_target_direction` expressed as an angle that depends on current ideal TO_TARGET vector
	ignition::math::Angle to_target_direction(std::atan2(to_target_vector.Y(), to_target_vector.X()));
	to_target_direction.Normalize();

	// transform actor orientation to world coordinate system
	ignition::math::Angle yaw_actor_w(pose_world_ptr_->Rot().Yaw() - IGN_PI_2);
	yaw_actor_w.Normalize();

	// compute angle difference
	ignition::math::Angle yaw_diff = to_target_direction - yaw_actor_w;
	yaw_diff.Normalize();

	// helper variable to determine the sign of the angular velocity that needs
	// to be applied to align with target direction
	int8_t sign = +1;

	// check the sign of the `yaw_diff` - the movement should be performed
	// in the SAME direction as the `yaw_diff` angle (shorter angular path)
	if ( yaw_diff.Radian() < 0.0 ) {
		sign = -1;
	}

	// save the `angle_change` which can be associated with angular displacement
	// applied to actor after this iteration
	double angle_change = 0.0;

	// smooth the rotation (limits angular velocity)
	static const double YAW_INCREMENT = 0.001;

	// calculate the angular displacement based on the angle difference (increment
	// or decrement actor's YAW)
	if ( std::fabs(yaw_diff.Radian()) < YAW_INCREMENT ) {
		angle_change = static_cast<double>(sign) * yaw_diff.Radian();
	} else {
		angle_change = static_cast<double>(sign) * YAW_INCREMENT;
	}

	// apply the change
	yaw_actor_w.Radian(yaw_actor_w.Radian() + angle_change);

	// transform actor `YAW` angle expressed in world coordinate system
	// to the angle expressed in actor coordinate system
	// NOTE: using the same variable, may be confusing
	yaw_actor_w.Radian(yaw_actor_w.Radian() + IGN_PI_2);
	yaw_actor_w.Normalize();

	// update actor pose with newly calculated `YAW` angle
	rpy->Z(yaw_actor_w.Radian());

	// Check how far from the perfect alignment the actor is.
	// True will be returned if the `yaw_diff` is small enough, false otherwise.
	yaw_diff.Radian(yaw_diff.Radian() + angle_change);
	yaw_diff.Normalize();

	// consider 10 degrees tolerance
	if ( std::fabs(yaw_diff.Radian()) < IGN_DTOR(10) ) {
		return (true);
	}
	return (false);

}

// ------------------------------------------------------------------- //

double Actor::prepareForUpdate() {

	// copy pose
	*pose_world_ptr_ = actor_ptr_->WorldPose();

	updateStanceOrientation(*pose_world_ptr_);

	double dt = (world_ptr_->SimTime() - time_last_update_).Double();
	velocity_.calculate(dt);

	common_info_ptr_->setLinearVel(velocity_.getLinear());

	// DELETE - the method used below just doesn't do anything - WorldPtr doesnt get updated
	// actor_ptr_->SetLinearVel(velocity_lin_);

	// update bounding model's pose
	updateBounding(*pose_world_ptr_);

	// dt is helpful for further calculations
	return (dt);

}

// ------------------------------------------------------------------- //

void Actor::applyUpdate(const double &dist_traveled) {

  	// save last position to calculate velocity
	*pose_world_prev_ptr_ = actor_ptr_->WorldPose();

	// make sure the actor won't go out of bounds
	if ( params_ptr_->getActorParams().limit_actors_workspace ) {
		pose_world_ptr_->Pos().X(std::max( params_ptr_->getActorParams().world_bound_x.at(0), std::min( params_ptr_->getActorParams().world_bound_x.at(1), pose_world_ptr_->Pos().X())));
		pose_world_ptr_->Pos().Y(std::max( params_ptr_->getActorParams().world_bound_y.at(0), std::min( params_ptr_->getActorParams().world_bound_y.at(1), pose_world_ptr_->Pos().Y())));
	}

	// update the global pose
	actor_ptr_->SetWorldPose(*pose_world_ptr_, false, false);


	/*
	 * std::cout << actor->GetName() << " | script time: " << actor->ScriptTime() << "\tdist_trav: " << _dist_traveled << "\tanim_factor: " << animation_factor << std::endl;
	 *
	 * for some reason (very likely some very small number returned as interaction force
	 * in social force model) dist_traveled sometimes turns out to be a NaN - one should then change
	 * it to a typical value of 0.005; NaN value of dist seems to be the cause of such error:
	 *
	 * "gazebo::common::NodeAnimation::FrameAt(double, bool) const: Assertion
	 *  `(t >= 0.0 && t <= 1.0)&&("t is not in the range 0.0..1.0")' failed."
	 *
	 *  NOTE: if such error occurs uncomment below line (not done for debugging process)
	 *  (std::isnan(_dist_traveled)) ? (_dist_traveled = 0.0005) : (0);
	 *
	 */
	// update script time to set proper animation speed
	actor_ptr_->SetScriptTime( actor_ptr_->ScriptTime() + (dist_traveled * params_ptr_->getActorParams().animation_factor) );

	// update time
	time_last_update_ = world_ptr_->SimTime();

	// this is useful whatever the state of the actor is
	visualizePositionData();

	// evaluate whether a status of the actual path container has changed
	if ( path_storage_.isUpdated() ) {

		// publish closest distance
		std_msgs::Float32 msg;
		msg.data = path_storage_.getDistance();
		stream_.publishData(ActorObstacleMsgType::ACTOR_OBSTACLE_DIST_CLOSEST, msg);
		// publish real path
		stream_.publishData(ActorNavMsgType::ACTOR_NAV_PATH_REAL, path_storage_.getPath());

	} else if ( path_storage_.isResetted() ) {

		// publish an empty path
		stream_.publishData(ActorNavMsgType::ACTOR_NAV_PATH_REAL, path_storage_.getPath());

	}


}

// ------------------------------------------------------------------- //

bool Actor::manageTargetMovingAround() {

	// check if call is executed for a proper mode
	if ( target_manager_.isFollowing() ) {
		// this should not happen
		return (false);
	}

	// helper flag
	bool new_target = false;

	// check whether a target exists
	if ( !target_manager_.isTargetChosen() ) {
		if ( target_manager_.changeTarget() ) {
			new_target = true;
		} else {
			setState(actor::ACTOR_STATE_STOP_AND_STARE);
			action_info_.setStatus(Action::MovingAroundStatus::FAILED);
		}
	}

	// check whether a target has plan generated
	if ( !target_manager_.isPlanGenerated() ) {
		if ( !target_manager_.generatePathPlan(pose_world_ptr_->Pos(), target_manager_.getTarget()) ) {
			target_manager_.abandonTarget();
			// do not change the `new_target` flag as the new one was not chosen;
			// what follows here is:
			// 1) 	this Gazebo Update event is omitted (no further action)
			// 2) 	at the start of the next Gazebo Update event isTargetChosen() returns false
			//  	immediately after that, a new target is chosen
			// 3) 	the Gazebo Update event is broken (no further action after target selection)
			// 4) 	at the start of the next Gazebo Update event isPlanGenerated() returns false
			// 		and a new global plan generation is started
			// 5) 	if global plan cannot be generated - move on to the 1) again
		}
	}

	// check whether a current checkpoint is abandonable
	if ( target_manager_.isCheckpointAbandonable() ) {
		target_manager_.updateCheckpoint();
	}

	// check closeness to the target/checkpoint
	if ( target_manager_.isTargetReached() ) {

		action_info_.setStatus(Action::MovingAroundStatus::GOAL_REACHED);

		// object tracking not active
		if ( target_manager_.changeTarget() ) {
			// after setting a new target, firstly let's rotate to its direction
			new_target = true;
		} else {
			setState(actor::ACTOR_STATE_STOP_AND_STARE);
			action_info_.setStatus(Action::MovingAroundStatus::FAILED);
		}

	} else if ( target_manager_.isCheckpointReached() ) {

		// take next checkpoint from vector (path)
		target_manager_.updateCheckpoint();

	}

	// reachability test 1: check if there has been some obstacle put into world since last target selection
	// reachability test 2: check if actor is stuck
	if ( !target_manager_.isTargetStillReachable() || target_manager_.isTargetNotReachedForTooLong() ) {

		target_manager_.abandonTarget();
		bool target_changed = false;
		// try a few times
		for ( size_t i = 0; i < 10; i++ ) {
			if ( target_manager_.changeTarget() ) {
				// after setting new target, first let's rotate to its direction
				// state will be changed in the next iteration
				new_target = true;
				target_changed = true;
				break;
			}
		}
		// new target was not found
		if ( !target_changed ) {
			setState(actor::ACTOR_STATE_STOP_AND_STARE);
			action_info_.setStatus(Action::MovingAroundStatus::FAILED);
			return (false);
		}

	}

	// evaluate whether a new target has been defined
	if ( new_target ) {
		// after selection of a new target, firstly let's rotate to its direction
		setState(actor::ACTOR_STATE_ALIGN_TARGET);
		action_info_.setStatus(Action::MovingAroundStatus::ROTATE_TOWARDS_OBJECT);
		return (false);
	}

	action_info_.setStatus(Action::MovingAroundStatus::APPROACHING);
	return (true);

}

// ------------------------------------------------------------------- //

bool Actor::manageTargetTracking() {

	// check if call is executed for a proper mode
	if ( !target_manager_.isFollowing() ) {
		action_info_.setStatus(Action::WRONG_STATE, "got into target tracking state but object following has not been enabled");
		return (false);
	}

	// helper flag
	bool stop_tracking = false;

	// flags helping in evaluation whether the tracked object started moving away again
	bool target_dir_alignment = false;	// whether to change the state to `ALIGN_WITH_TARGET_DIR`
	bool object_prev_reached = false; 	// by default the tracked object moves away
	if ( target_manager_.isFollowedObjectReached() ) {
		// let's save the previous status before performing any further
		// evaluations (for example `isTargetReached()`)
		object_prev_reached = true; // tracked object has been stopped so far
	}

	// try to update a global path leading the actor towards the dynamic object;
	// this also checks whether an object to follow is selected
	if ( !target_manager_.updateFollowedTarget() ) {
		// recently followed object is not reachable anymore (object deleted
		// from the simulation or global plan cannot be found)
		if ( follow_object_.doWait(world_ptr_->SimTime()) ) {

			action_info_.setStatus(Action::FollowObjectStatus::WAIT_FOR_MOVEMENT, "tracked object is not reachable now, waiting...");
			// make actor stop;
			// when tracked object will start moving again, then stance will be changed
			setStance(ActorStance::ACTOR_STANCE_STAND);
			// update the pose (stance only) because the update event will be broken (stopped)
			updateStanceOrientation(*pose_world_ptr_);
			applyUpdate(0.001);
			return (false);

		} else {
			// a number of tries were performed with no luck - follow object
			// state will be terminated
			stop_tracking = true;
			action_info_.setStatus(Action::FollowObjectStatus::UNABLE_TO_FIND_PLAN, "tracked object is not reachable anymore (has been deleted from the world or a global plan cannot be generated)");
		}
	} else if ( !object_prev_reached && (action_info_.getStatus() != static_cast<int>(Action::FollowObjectStatus::WAIT_FOR_MOVEMENT)) ) {
		// global path was updated successfully
		action_info_.setStatus(Action::FollowObjectStatus::TRACKING, "tracking");
	}

	// check closeness to the target/checkpoint
	if ( target_manager_.isTargetReached() && (action_info_.getStatus() != static_cast<int>(Action::FollowObjectStatus::WAIT_FOR_MOVEMENT)) ) {
		// actor is close enough to the tracked object (it may not be moving for some time);
		// do not call stopFollowing etc and do not change the state,
		// just do not try to go further at the moment
		action_info_.setStatus(Action::FollowObjectStatus::OBJECT_REACHED, "target reached");
	} else if ( object_prev_reached ) {
		// detects change of the `is_followed_object_reached_`
		// flag (i.e. tracked object started moving again)
		target_dir_alignment = true;
	} else if ( target_manager_.isCheckpointReached() ) {
		// take next checkpoint from vector (path)
		target_manager_.updateCheckpoint();
		action_info_.setStatus(Action::FollowObjectStatus::TRACKING, "tracking");
	}

	// check whether a current checkpoint is abandonable (angle-related)
	if ( target_manager_.isCheckpointAbandonable() ) {
		target_manager_.updateCheckpoint();
		action_info_.setStatus(Action::FollowObjectStatus::TRACKING, "tracking");
	}

	// check if the tracked object still exists in the world since last target selection
	if ( !target_manager_.isTargetStillReachable() ) {
		stop_tracking = true;
		action_info_.setStatus(Action::FollowObjectStatus::NON_REACHABLE, "the tracked object became unreachable");
	}

	// change FSM state if needed
	if ( stop_tracking ) {
		action_info_.terminate();
		target_manager_.abandonTarget();
		target_manager_.stopFollowing();
		ignored_models_.pop_back(); // FIXME: it won't work if ignored_models stores other elements than followed object's name
		setState(ActorState::ACTOR_STATE_STOP_AND_STARE);
		resetVisualization();
		return (false);
	}

	// dynamic target has been reached, do not change the state, just change stance
	if ( target_manager_.isFollowedObjectReached() // ||
		//(action_info_.getStatus() == actor::core::Action::WAIT_FOR_MOVEMENT)
		)
	{
		// TODO: the best would be to separate these 2 conditions as for ObjectReached there should
		// such debug info: "tracked object is within 'reachment' tolerance range"
		// whereas for WAIT FOR MOVEMENT:
		// "tracked object is not reachable now, waiting...")
		// NOT VALID ATM since the part shown below was copied below the 'followObjectDoWait' call

		// make actor stop;
		// when tracked object will start moving again, then stance will be changed
		setStance(ActorStance::ACTOR_STANCE_STAND);
		// update the pose (stance only) because the update event will be broken (stopped)
		updateStanceOrientation(*pose_world_ptr_);
		applyUpdate(0.001);
		return (false);
	}

	// target previously was reached but started moving again - let's align
	// with direction to its center
	if ( target_dir_alignment ) {
		action_info_.setStatus(Action::FollowObjectStatus::ROTATE_TOWARDS_OBJECT, "aligning actor face direction with the direction to a center of the tracked object");
		setStance(ActorStance::ACTOR_STANCE_WALK);
		setState(ActorState::ACTOR_STATE_ALIGN_TARGET);
		return (false);
	}

	return (true);

}

// ------------------------------------------------------------------- //

bool Actor::manageTargetSingleReachment() {

	// check if call is executed for a proper mode
	if ( target_manager_.isFollowing() ) {
		action_info_.setStatus(actor::core::Action::SetGoalStatus::FOLLOWING, "wrong state handler, finishing");
		return (false);
	}

	// helper flag
	bool abandon = false;

	// check whether a target exists
	if ( !target_manager_.isTargetChosen() ) {
		//return (false);
		abandon = true;
		action_info_.setStatus(actor::core::Action::SetGoalStatus::GOAL_NOT_SELECTED, "target manager has no target chosen");
	}

	// check whether a target has plan generated
	if ( !target_manager_.isPlanGenerated() ) {
		if ( !target_manager_.generatePathPlan(pose_world_ptr_->Pos(), target_manager_.getTarget()) ) {
			abandon = true;
			action_info_.setStatus(Action::SetGoalStatus::NON_REACHABLE, "the selected goal is not reachable");
		}
	}

	// check whether a current checkpoint is abandonable
	if ( target_manager_.isCheckpointAbandonable() ) {
		target_manager_.updateCheckpoint();
		action_info_.setStatus(Action::SetGoalStatus::APPROACHING, "approaching to the goal");
	}

	// check closeness to the target/checkpoint
	if ( target_manager_.isTargetReached() ) {
		action_info_.setStatus(Action::SetGoalStatus::GOAL_REACHED, "goal has been reached successfully!");
		abandon = true;
	} else if ( target_manager_.isCheckpointReached() ) {
		// take next checkpoint from vector (path)
		target_manager_.updateCheckpoint();
		action_info_.setStatus(Action::SetGoalStatus::APPROACHING, "approaching to the goal");
	}

	// reachability test 1:
	// check if there has been some obstacle put into world since last target selection
	if ( !target_manager_.isTargetStillReachable() ) {
		action_info_.setStatus(Action::SetGoalStatus::NON_REACHABLE, "target became unreachable");
		abandon = true;
	}

	// reachability test 2:
	// check if actor is stuck
	if ( target_manager_.isTargetNotReachedForTooLong() ) {
		action_info_.setStatus(Action::SetGoalStatus::NON_REACHABLE, "actor struggles to reach the goal for too long, terminating...");
		abandon = true;
	}

	// do abandon the current target? either reached or non-reachable anymore
	if ( abandon ) {
//		target_manager_.abandonTarget();
//		setState(actor::ACTOR_STATE_STOP_AND_STARE);
		return (false);
	}

	return (true);

}

// ------------------------------------------------------------------- //

double Actor::move(const double &dt) {

	// calculate `social` force (i.e. `internal` and `interaction` components)
	sfm_.computeSocialForce(world_ptr_, *pose_world_ptr_, velocity_.getLinear(),
						    target_manager_.getCheckpoint(),
						    *common_info_ptr_.get(), dt, ignored_models_);

	// actual `social` vector
	ignition::math::Vector3d human_action_force(0.0, 0.0, 0.0);

	// evaluate whether more complex forces are supposed to be calculated
	if ( !params_ptr_->getSfmParams().disable_interaction_forces ) {

		// execute fuzzy operations block
		fuzzy_processor_.load(sfm_.getDirectionAlpha(), sfm_.getDirectionBetaDynamic(),
							  sfm_.getRelativeLocationDynamic(), sfm_.getDistanceAngleDynamic());
		fuzzy_processor_.process();

		// create a force vector according to the activated `social behaviour`
		social_conductor_.apply(sfm_.getForceCombined(), sfm_.getDirectionAlpha(), sfm_.getDistanceDynamic(),
								fuzzy_processor_.getOutput());

		// assign `social` vector
		human_action_force = social_conductor_.getSocialVector();

	}

    // according to the force, calculate a new pose
	ignition::math::Pose3d new_pose = sfm_.computeNewPose(*pose_world_ptr_, velocity_.getLinear(),
														  sfm_.getForceCombined() + human_action_force,
														  target_manager_.getCheckpoint(), dt);

	// object info update
	double dist_traveled = (new_pose.Pos() - actor_ptr_->WorldPose().Pos()).Length();

	// update the local copy of the actor's pose
	*pose_world_ptr_ = new_pose;

	// collect data to visualize actor's path
	path_storage_.collect(pose_world_ptr_->Pos(), sfm_.getDistanceClosestStaticObstacle());

	return (dist_traveled);

}

// ------------------------------------------------------------------- //

void Actor::updateBounding(const ignition::math::Pose3d &pose) {

	/* update the bounding box/circle/ellipse of the actor
	 * (aim is to create a kind of an inflation layer) */
	switch ( bounding_type_ ) {

	case(ACTOR_BOUNDING_BOX):
			bounding_ptr_->updatePose(pose);
			break;

	case(ACTOR_BOUNDING_CIRCLE):
			bounding_ptr_->updatePose(pose);
			break;

	case(ACTOR_BOUNDING_ELLIPSE):
			// correct yaw angle to make ellipse abstract from Actor coordinate system's orientation
			ignition::math::Angle yaw_world( pose.Rot().Yaw() - IGN_PI_2);
			yaw_world.Normalize();
			bounding_ptr_->updatePose(ignition::math::Pose3d( pose.Pos(),
									  ignition::math::Quaterniond(pose.Rot().Roll(),
																pose.Rot().Pitch(),
																yaw_world.Radian()) ));
			break;

	}

}

// ------------------------------------------------------------------- //

void Actor::updateTransitionFunctionPtr() {

	// print the name of the actor whose state changes
	std::cout << "[ FSM ] " << actor_ptr_->GetName() << "'s \tnew state:";

	switch( fsm_.getState() ) {

	case(ACTOR_STATE_ALIGN_TARGET):
			std::cout << "\talignToTarget" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerAlignTarget; 		// &this->Actor::stateHandlerAlignTarget;
			setStance(ACTOR_STANCE_WALK);
			break;
	case(ACTOR_STATE_STUCK):
			std::cout << "\tgotStuck" << std::endl;
			// empty
			break;
	case(ACTOR_STATE_MOVE_AROUND):
			std::cout << "\tmoveAround" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerMoveAround; 		// &this->Actor::stateHandlerMoveAround;
			setStance(ACTOR_STANCE_WALK);
			break;
	case(ACTOR_STATE_TARGET_REACHING):
			std::cout << "\ttargetReaching" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerTargetReaching;
			setStance(ACTOR_STANCE_WALK);
			break;
	case(ACTOR_STATE_LIE_DOWN):
			std::cout << "\tlieDown" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerLieDown;
			setStance(ACTOR_STANCE_WALK);
			break;
	case(ACTOR_STATE_STOP_AND_STARE):
			std::cout << "\tstopAndStare" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerStopAndStare;
			setStance(ACTOR_STANCE_STAND);
			// empty
			break;
	case(ACTOR_STATE_FOLLOW_OBJECT):
			std::cout << "\tfollowObject" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerFollowObject; 	// &this->Actor::stateHandlerFollowObject;
			setStance(ACTOR_STANCE_WALK);
			break;
	case(ACTOR_STATE_TELEOPERATION):
			std::cout << "\tteleoperation" << std::endl;
			trans_function_ptr = &actor::core::Actor::stateHandlerTeleoperation;
			setStance(ACTOR_STANCE_STAND);
			break;
	default:
			std::cout << "\tUNKNOWN" << std::endl;
			break;

	}

}

// ------------------------------------------------------------------- //

std::string Actor::convertStanceToAnimationName() const {

	std::string anim_name;

	switch( stance_ ) {

	case(ACTOR_STANCE_WALK):
			anim_name = "walk";
			break;
	case(ACTOR_STANCE_STAND):
			anim_name = "stand";
			break;
	case(ACTOR_STANCE_LIE):
			anim_name = "stand"; // lie is stand but in different plane
			break;
	case(ACTOR_STANCE_SIT_DOWN):
			anim_name = "sit_down";
			break;
	case(ACTOR_STANCE_SITTING):
			anim_name = "sitting";
			break;
	case(ACTOR_STANCE_STAND_UP):
			anim_name = "stand_up";
			break;
	case(ACTOR_STANCE_RUN):
			anim_name = "run";
			break;
	case(ACTOR_STANCE_TALK_A):
			anim_name = "talk_a";
			break;
	case(ACTOR_STANCE_TALK_B):
			anim_name = "talk_b";
			break;
	default:
			break;

	}

	return (anim_name);

}

// ------------------------------------------------------------------- //

void Actor::visualizePositionData() {

	// Hard-coded 30 Hz refresh rate
	if ( (world_ptr_->SimTime() - time_last_tf_pub_).Double() >= 0.033 ) {

		// update time stamp
		time_last_tf_pub_ = world_ptr_->SimTime();

		// publish data for visualization
		stream_.publishData(ActorMarkerType::ACTOR_MARKER_BOUNDING, bounding_ptr_->getMarkerConversion());

		stream_.publishData(ActorTfType::ACTOR_TF_SELF, *pose_world_ptr_);
		stream_.publishData(ActorTfType::ACTOR_TF_TARGET, ignition::math::Pose3d(ignition::math::Vector3d(target_manager_.getTarget()),
																				 ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)));
		stream_.publishData(ActorTfType::ACTOR_TF_CHECKPOINT, ignition::math::Pose3d(ignition::math::Vector3d(target_manager_.getCheckpoint()),
																				 ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)));

	}

	// publish path if a new one has just been generated
	if ( target_manager_.isPathNew() ) {
		stream_.publishData(ActorNavMsgType::ACTOR_NAV_PATH_GLOBAL, target_manager_.getPath());
	}

}

// ------------------------------------------------------------------- //

void Actor::visualizeSfmCalculations() {

	// publish social force vector and closest points lines @ 30 Hz
	if ( (world_ptr_->SimTime() - time_last_vis_sfm_pub_).Double() >= 0.033 ) {

		// save last event time
		time_last_vis_sfm_pub_ = world_ptr_->SimTime();

		// combined vector - red
		sfm_vis_arrow_.setColor(1.0f, 0.0f, 0.0f, 1.0f);
		stream_.publishData(ActorMarkerType::ACTOR_MARKER_COMBINED_VECTOR, 			 sfm_vis_arrow_.create(ignition::math::Vector3d(pose_world_ptr_->Pos().X(), pose_world_ptr_->Pos().Y(), 0.0), sfm_.getForceCombined()) );

		// internal force vector - green
		sfm_vis_arrow_.setColor(0.0f, 1.0f, 0.0f, 1.0f);
		stream_.publishData(ActorMarkerType::ACTOR_MARKER_INTERNAL_VECTOR, 			 sfm_vis_arrow_.create(ignition::math::Vector3d(pose_world_ptr_->Pos().X(), pose_world_ptr_->Pos().Y(), 0.2), sfm_.getForceInternal()) );

		// interaction force vector - cyan
		sfm_vis_arrow_.setColor(0.0f, 0.8f, 1.0f, 1.0f);
		stream_.publishData(ActorMarkerType::ACTOR_MARKER_INTERACTION_VECTOR, 		 sfm_vis_arrow_.create(ignition::math::Vector3d(pose_world_ptr_->Pos().X(), pose_world_ptr_->Pos().Y(), 0.4), sfm_.getForceInteraction()) );

		// social force vector - orange
		sfm_vis_arrow_.setColor(1.0f, 0.6f, 0.0f, 1.0f);
		stream_.publishData(ActorMarkerType::ACTOR_MARKER_SOCIAL_VECTOR, 			 sfm_vis_arrow_.create(ignition::math::Vector3d(pose_world_ptr_->Pos().X(), pose_world_ptr_->Pos().Y(), 0.6), social_conductor_.getSocialVector()) );

		stream_.publishData(ActorMarkerTextType::ACTOR_MARKER_TEXT_BEH, 			 sfm_vis_text_.create( ignition::math::Vector3d(pose_world_ptr_->Pos().X(), pose_world_ptr_->Pos().Y(), 1.0), social_conductor_.getBehaviourActive()) );
		stream_.publishData(ActorMarkerArrayType::ACTOR_MARKER_ARRAY_CLOSEST_POINTS, sfm_vis_line_list_.createArray(sfm_.getClosestPointsVector()) );

	}

	// check if grid publication has been enabled in the parameter file
	if ( params_ptr_->getSfmVisParams().publish_grid ) {

		/* visualize SFM grid if enough time elapsed from last
		 * publication and there is some unit subscribing to a topic */
		if ( visualizeVectorField() ) {
			stream_.publishData( actor::ActorMarkerArrayType::ACTOR_MARKER_ARRAY_GRID, sfm_vis_grid_.getMarkerArray() );
		}

	}

	// check if potential field publication has been enabled in the parameter file
	if ( params_ptr_->getSfmVisParams().publish_potential ) {
		if ( visualizeHeatmap() ) {
			stream_.publishData( actor::ActorMarkerArrayType::ACTOR_MARKER_ARRAY_POTENTIAL, sfm_vis_heatmap_.getMarkerArray() );
		}
	}

}

// ------------------------------------------------------------------- //

bool Actor::visualizeVectorField() {

	// do not publish too often
	if ( (world_ptr_->SimTime() - time_last_vis_grid_pub_).Double() > params_ptr_->getSfmVisParams().grid_pub_period ) {

		/* update the sim time even when grid will not be published
		 * to avoid calling getSubscribersNum() in each iteration */
		time_last_vis_grid_pub_ = world_ptr_->SimTime();

		/* creating a grid with high resolution is pretty time-consuming
		 * check if there is a subscriber and then calculate force vectors
		 * for a whole grid */

		/* grid generation is orientation-dependent (current orientation
		 * of an actor is used) */
		if ( stream_.getSubscribersNum(actor::ActorMarkerArrayType::ACTOR_MARKER_ARRAY_GRID ) ) {

			ignition::math::Pose3d pose;	// pose where `virtual` actor will be placed in

			// before a start reset a grid index
			sfm_vis_grid_.resetGridIndex();

			while ( !sfm_vis_grid_.isWholeGridChecked() ) {

				// set an actor's virtual pose
				pose = ignition::math::Pose3d( sfm_vis_grid_.getNextGridElement(), pose_world_ptr_->Rot() );

				// update the bounding of an actor
				updateBounding(pose);

				// calculate social force for actor located in current pose
				// hard-coded time delta
				sfm_.computeSocialForce(world_ptr_, pose, velocity_.getLinear(), target_manager_.getCheckpoint(), *common_info_ptr_.get(), 0.001, ignored_models_);

				// pass a result to vector of grid forces
				sfm_vis_grid_.addMarker( sfm_vis_grid_.create(pose.Pos(), sfm_.getForceCombined()) );

			}
			return (true);

		} /* getSubscribersNum() */

	} /* if ( time_elapsed ) */

	return (false);

}

// ------------------------------------------------------------------- //

bool Actor::visualizeHeatmap() {

	// do not publish too often
	if ( (world_ptr_->SimTime() - time_last_vis_potential_pub_).Double() > params_ptr_->getSfmVisParams().grid_pub_period ) {

		/* update the sim time even when grid will not be published
		 * to avoid calling getSubscribersNum() in each iteration */
		time_last_vis_potential_pub_ = world_ptr_->SimTime();

		/* creating a grid with high resolution is pretty time-consuming
		 * check if there is a subscriber and then calculate force vectors
		 * for a whole grid */

		/* grid generation is orientation-dependent (current orientation
		 * of an actor is used) */
		if ( stream_.getSubscribersNum(actor::ActorMarkerArrayType::ACTOR_MARKER_ARRAY_POTENTIAL ) ) {

			ignition::math::Pose3d pose;	// pose where `virtual` actor will be placed in

			// before a start reset a grid index
			sfm_vis_heatmap_.resetGridIndex();

			while ( !sfm_vis_heatmap_.isWholeGridChecked() ) {

				// set an actor's virtual pose
				pose = ignition::math::Pose3d( sfm_vis_heatmap_.getNextGridElement(), pose_world_ptr_->Rot() );
				pose.Rot().Euler(0.0, 0.0, 0.0);

				// update the bounding of an actor
				updateBounding(pose);

				// calculate social force for actor located in current pose
				// hard-coded time delta
				sfm_.computeSocialForce(world_ptr_, pose, velocity_.getLinear(), target_manager_.getCheckpoint(), *common_info_ptr_.get(), params_ptr_->getSfmVisParams().markers_pub_period, ignored_models_);

				// pass a result to vector of grid forces
				sfm_vis_heatmap_.addMarker(sfm_vis_heatmap_.create(pose.Pos(), sfm_.getForceInteraction().Length()));

			}
			return (true);

		} /* getSubscribersNum() */

	} /* if ( time_elapsed ) */

	return (false);

}

// ------------------------------------------------------------------- //

void Actor::resetVisualization(bool reset_global_path) {
	if ( reset_global_path ) {
		target_manager_.resetPath();
	}
	sfm_.reset(); // clear SFM markers
	social_conductor_.reset();
	path_storage_.reset();
}

// ------------------------------------------------------------------- //

Actor::~Actor() { }

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
