/*
 * SocialForceModel.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#include "sfm/core/SocialForceModel.h"
#include <cmath>		// atan2()
#include <tgmath.h>		// fabs()
#include <tuple>		// std::tuple

#include "sfm/core/SFMDebug.h"

// ----------------------------------------

#include "BoundingEllipseDebug.h"

namespace sfm {
namespace core {

// - - - - - - - - - - - - - - - - -

//#include "../../print_info.h"
static bool print_info = false;
static int print_counter = 0;

// ------------------------------------------------------------------- //

// TODO: add references to arguments to avoid copy

// ------------------------------------------------------------------- //

/* TODO: from ActorPlugin HandleObstacles:
if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
{

}
*/

// ------------------------------------------------------------------- //

SocialForceModel::SocialForceModel():

	/* 		TUNING TIPS:
	 * 	o 	the higher the desired force factor set the more actor will not notice
	 * 		surrounding obstacles, trying to achieve a goal as fast as possible
	 * 	o	the higher the interaction force factor set the more actor will try to
	 * 		avoid obstacles - he will pass them safer
	 * 	o 	the higher the yaw_increment coefficient set the less inertia actor will have
	 * 		and will react immediately to social force changes - this will lead to many
	 * 		rotations; on the other hand setting this parameter too low will create
	 * 		a very conservative movement style (possibly cause stepping into obstacles
	 * 		in cluttered world) */

	fov(2.00), speed_max(1.50), yaw_max_delta(20.0 / M_PI * 180.0), mass_person(1),
	desired_force_factor(100.0), // desired_force_factor(200.0),
	interaction_force_factor(3000.0), // interaction_force_factor(6000.0),
	force_max(2000.0), force_min(300.0), // force_min(800.0)
	inflation_type(INFLATION_ELLIPSE),
	interaction_static_type(INTERACTION_ELLIPTICAL),
	param_description_(PARAMETER_DESCRIPTION_2014)

{

	SetParameterValues();

	closest_points.clear();

	/* Algorithm PARAMETERS are:
	 * - relaxation time must be given here
	 * - kind of coefficient for attraction artificial potential field (decreases over time)
	 * - FOV
	 * - direction weight
	 * - max speed
	 */

}

// ------------------------------------------------------------------- //

std::vector<ignition::math::Pose3d> SocialForceModel::GetClosestPointsVector() const {
	return (this->closest_points);
}

// ------------------------------------------------------------------- //

void SocialForceModel::Init(const unsigned short int _mass_person,
		const float _desired_force_factor,
		const float _interaction_force_factor,
		const gazebo::physics::WorldPtr _world_ptr) {

	// TODO: discard the objects that should be ignored
	for ( unsigned int i = 0; i < _world_ptr->ModelCount(); i++ ) {
		map_models_rel_locations[_world_ptr->ModelByIndex(i)->GetName()] = LOCATION_UNSPECIFIED;
	}

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetSocialForce(
	const gazebo::physics::WorldPtr _world_ptr,
	const std::string _actor_name,
	const ignition::math::Pose3d _actor_pose,
	const ignition::math::Vector3d _actor_velocity,
	const ignition::math::Vector3d _actor_target,
	const actor::core::CommonInfo &_actor_info)

{

#ifdef DEBUG_LOG_ALL_INTERACTIONS
	std::stringstream log_msg; // debug
#endif

	closest_points.clear();

	( SfmGetPrintData() ) ? (print_info = true) : (0);

//	if ( _actor_name == "actor1" && SfmGetPrintData()) {
//		debugEllipseSet(true);
//	} else {
//		debugEllipseSet(false);
//	}


	// compute internal acceleration
	ignition::math::Vector3d f_alpha = GetInternalAcceleration(_actor_pose, _actor_velocity, _actor_target);

	// allocate variables needed in loop
	ignition::math::Vector3d f_interaction_total(0.0, 0.0, 0.0);
	ignition::math::Vector3d f_alpha_beta(0.0, 0.0, 0.0);

#ifdef CALCULATE_INTERACTION

	/* model_vel contains model's velocity (world's object or actor) - for the actor this is set differently
	 * it was impossible to set actor's linear velocity by setting it by the model's class method */
	ignition::math::Vector3d model_vel;
	actor::inflation::Box model_box;
	actor::inflation::Circle model_circle;
	actor::inflation::Ellipse model_ellipse;

	/* below flag is used as a workaround for the problem connected with being unable to set actor's
	 * velocity and acceleration in the gazebo::physics::WorldPtr */
	bool is_an_actor = false;
	gazebo::physics::ModelPtr model_ptr;

	// iterate over all world's objects
	for ( unsigned int i = 0; i < _world_ptr->ModelCount(); i++ ) {

		model_ptr = _world_ptr->ModelByIndex(i);

		if ( model_ptr->GetName() == _actor_name ) {
			// do not calculate social force from itself
			continue;
		}

		// test world specific names
		// FIXME
		if ( model_ptr->GetName() == "cafe" || model_ptr->GetName() == "ground_plane" ) {
			// do not calculate social force from the objects he is stepping on
			continue;
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////
		// catch especially table1 - debugging ///////////////////////////////////////////////////////////////
		SfmDebugSetCurrentObjectName(model_ptr->GetName()); ////////////////////////////////////////////////////
		SfmDebugSetCurrentActorName(_actor_name); //////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////////

		if ( print_info ) {
			std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
		}

		if ( (is_an_actor = actor_decoder.isActor(model_ptr->GetType())) ) {

			// decoder of the CommonInfo class
			actor_decoder.setID(model_ptr->GetName(), _actor_info.getNameIDMap());

			// load data from CommonInfo based on actor's id
			model_vel = actor_decoder.getData(_actor_info.getLinearVelocitiesVector());

			// select proper inflation model
			if ( inflation_type == INFLATION_CIRCLE ) {
				model_circle  = actor_decoder.getData(_actor_info.getBoundingCirclesVector());
			} else if ( inflation_type == INFLATION_ELLIPSE ) {
				model_ellipse = actor_decoder.getData(_actor_info.getBoundingEllipsesVector());
			} else if ( inflation_type == INFLATION_BOX_ALL_OBJECTS || inflation_type == INFLATION_BOX_OTHER_OBJECTS ) {
				model_box = actor_decoder.getData(_actor_info.getBoundingBoxesVector());
			}

		} else {

			model_vel = model_ptr->WorldLinearVel();
			model_box.setBox(model_ptr->BoundingBox()); // conversion

		}

		// ============================================================================

		// model_closest i.e. closest to an actor or the actor's bounding
		ignition::math::Pose3d actor_closest_to_model_pose;
		ignition::math::Pose3d model_closest_point_pose;

//		std::cout << "START DEBUGGING\n\t" << SfmDebugGetCurrentActorName() << "\t\t" << SfmDebugGetCurrentObjectName() << std::endl;
//		std::cout << "\tinitial actor_pose: " << _actor_pose << "\tmodel_pose: " << model_ptr->WorldPose() << std::endl;

		// calculate closest points
		switch(inflation_type) {

		case(INFLATION_BOX_OTHER_OBJECTS):

//				std::cout << "\tINFLATION - BOX - OTHER OBJECTS" << std::endl;
				actor_closest_to_model_pose = _actor_pose;
				model_closest_point_pose.Pos() = inflate.findModelsClosestPoints(_actor_pose, model_ptr->WorldPose(), model_box);
				break;

		case(INFLATION_BOX_ALL_OBJECTS):

//				std::cout << "\tINFLATION - BOX - ALL OBJECTS" << std::endl;
				std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
						inflate.findModelsClosestPoints(_actor_pose, _actor_info.getBoundingBox(),
															 model_ptr->WorldPose(), model_box, model_ptr->GetName() );
				break;

		case(INFLATION_CIRCLE):

				if ( is_an_actor ) {
//					std::cout << "\tINFLATION - CIRCLE - actor" << std::endl;
					std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
							inflate.findModelsClosestPoints(_actor_pose, _actor_info.getBoundingCircle(),
																 model_ptr->WorldPose(), model_circle, model_ptr->GetName() );
				} else {
//					std::cout << "\tINFLATION - CIRCLE - non-actor" << std::endl;
					std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
							inflate.findModelsClosestPoints(_actor_pose, _actor_info.getBoundingCircle(),
																 model_ptr->WorldPose(), model_box, model_ptr->GetName() );
				}
				break;

		case(INFLATION_ELLIPSE):

				if ( is_an_actor ) {
//					std::cout << "\tINFLATION - ELLIPSE - actor" << std::endl;
					std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
							inflate.findModelsClosestPoints(_actor_pose, _actor_info.getBoundingEllipse(),
																 model_ptr->WorldPose(), model_ellipse, model_ptr->GetName() );
				} else {
//					std::cout << "\tINFLATION - ELLIPSE - non actor" << std::endl;
					std::tie( actor_closest_to_model_pose, model_closest_point_pose.Pos() ) =
							inflate.findModelsClosestPoints(_actor_pose, _actor_info.getBoundingEllipse(),
																 model_ptr->WorldPose(), model_box, model_ptr->GetName() );

				}
				break;

		default:

				// no inflation
//				std::cout << "\tINFLATION - DEFAULT" << std::endl;
				actor_closest_to_model_pose = _actor_pose;
				model_closest_point_pose = model_ptr->WorldPose();
				break;

		}

//		std::cout << "\tFINAL actor_pose: " << actor_closest_to_model_pose << "\tmodel_pose: " << model_closest_point_pose << std::endl;

		// debug txt
		if ( print_info ) { std::cout << "actor_center: " << _actor_pose.Pos() << "\tobstacle_closest_pt: " << model_closest_point_pose.Pos() << "\tdist: " << (model_closest_point_pose.Pos()-_actor_pose.Pos()).Length() << std::endl; }

		// debug closest points
		closest_points.push_back(model_closest_point_pose);
		closest_points.push_back(actor_closest_to_model_pose);

		// based on a parameter and an object type - calculate a force from a static object properly
		if ( is_an_actor || interaction_static_type == INTERACTION_REPULSIVE_EVASIVE ) {

//			std::cout << "\tf_alpha_beta - NON STATIC" << std::endl;
			// calculate interaction force
			f_alpha_beta = GetInteractionComponent(	actor_closest_to_model_pose, _actor_velocity,
													model_closest_point_pose, model_vel, is_an_actor);

		} else {

//			std::cout << "\tf_alpha_beta - STATIC" << std::endl;
			// FIXME: dt arbitrary ATM!
			f_alpha_beta = GetForceFromStaticObstacle(actor_closest_to_model_pose, _actor_velocity,
													  model_closest_point_pose, 0.001);

		}

//		std::cout << "\n\n\n" << std::endl;

		// ============================================================================

#ifdef DEBUG_OSCILLATIONS
		if ( _actor_name == "actor1" && model_ptr->GetName() == "table1" ) {
			std::cout << "\tactor1-table1 interaction vector: " << f_alpha_beta.X() << "\t" << f_alpha_beta.Y() << std::endl;
		}
#endif

		/* Kind of a hack connected with very strong repulsion when actors are close to each other
		 * whereas in bigger distances the force is quite weak;
		 * Truncate very big forces from single objects */
		if ( is_an_actor ) {
			f_alpha_beta *= 0.50;
		} else {
			f_alpha_beta *= 1.75;
		}

		// truncate if force is too big -> causes immediate speed-up
		// or `sliding` when rotation smoothing is disabled (getNewPose())
		if ( f_alpha_beta.Length() > 1000.0 ) {
			f_alpha_beta = f_alpha_beta.Normalized() * 1000.0;
		}

		// sum all forces
		f_interaction_total += f_alpha_beta;
		if ( print_info ) {
			std::cout << " model's name: " << model_ptr->GetName() << "  pose: " << model_ptr->WorldPose() << "  lin vel: " << model_vel << "  force: " << f_alpha_beta << std::endl;
		}

#ifdef DEBUG_LOG_ALL_INTERACTIONS
		if ( SfmGetPrintData() ) {
		log_msg << "\t" << model_ptr->GetName();
		log_msg << "\t" << f_alpha_beta * interaction_force_factor << "\n";
		}
#endif

	} // for

#ifdef DEBUG_LOG_ALL_INTERACTIONS
	if ( SfmGetPrintData() ) {
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "**************************************************************************\n";
		std::cout << "LOG_MESSAGE - ALL OBJECTS ----- " << SfmDebugGetCurrentActorName() << std::endl;
		std::cout << "\tInternal: " << desired_force_factor * f_alpha << std::endl;
		std::cout << log_msg.str() << std::endl;
		std::cout << "\tTOTAL FORCE: " << desired_force_factor * f_alpha + interaction_force_factor * f_interaction_total << std::endl;
		std::cout << "**************************************************************************\n\n\n";
	}
	}
#endif

	if ( print_info ) {
		std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
	}
#endif



	/* TODO: set desired force factor according to the distance to the closest obstacle -
	 * the closer actor gets, the smaller the coefficient should be - this is
	 * the REAL SOCIAL feature of the model */
	// this->calculateDesiredForceFactor(dist_to_closest_obstacle);
	// inside the function lets normalize the factor according to the `desired` one

	ignition::math::Vector3d f_total = desired_force_factor * f_alpha + interaction_force_factor * f_interaction_total;
	f_total.Z(0.0);

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	( SfmGetPrintData() ) ? (print_info = true) : (0);
#endif

	if ( print_info ) {
		std::cout << "-----------------------\n";
		std::cout << _actor_name << " | SocialForce: " << f_total << "\tinternal: " << desired_force_factor * f_alpha << "\tinteraction: " << interaction_force_factor * f_interaction_total;
	}
//	std::cout << _actor_name << " | SocialForce: " << f_total << "\tinternal: " << desired_force_factor * f_alpha << "\tinteraction: " << interaction_force_factor * f_interaction_total;

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	( SfmGetPrintData() ) ? (print_info = false) : (0);
#endif

	// truncate the force value to max to prevent strange speedup of an actor
	double force_length = f_total.Length();
	if ( force_length > force_max ) {

		f_total = f_total.Normalize() * force_max;

		if ( print_info ) {
			std::cout << "\tTRUNCATED";
		}

	} /* */ else if ( force_length < force_min ) {

		f_total = f_total.Normalize() * force_min;

		if ( print_info ) {
			std::cout << "\tEXTENDED";
		}

	}

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	( SfmGetPrintData() ) ? (print_info = true) : (0);
#endif

	if ( print_info ) {
		std::cout << "\n" << SfmDebugGetCurrentActorName() << " | finalValue: " << f_total << "\tlength: " << f_total.Length() << std::endl;
	}

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	( SfmGetPrintData() ) ? (print_info = false) : (0);
#endif

	return (f_total);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetInternalAcceleration(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_vel,
		const ignition::math::Vector3d &_actor_target)
{

	// TODO: a lot of allocations here
	ignition::math::Vector3d to_goal_vector = (_actor_target - _actor_pose.Pos());
	ignition::math::Vector3d to_goal_direction = to_goal_vector.Normalize();
	ignition::math::Vector3d ideal_vel_vector = speed_desired * to_goal_direction;
	ignition::math::Vector3d f_alpha = mass_person * (1/relaxation_time) * (ideal_vel_vector - _actor_vel);
	f_alpha.Z(0.0);

#ifdef DEBUG_INTERNAL_ACC
//	if ( print_info ) {
		std::cout << std::endl;
		std::cout << "---------------------------------------------------------------------------------\n";
		std::cout << "GetInternalAcceleration() - " << SfmDebugGetCurrentActorName() << std::endl;
		std::cout << "\tactor_pos: " << _actor_pose.Pos();
		std::cout << "\ttarget: " << _actor_target << "   to_goal_direction: " << to_goal_direction;
		std::cout << "\n\tactor_vel: " << _actor_vel << "\tideal_vel_vector: " << ideal_vel_vector;
		std::cout << "\tf_alpha: " << f_alpha * desired_force_factor;
		std::cout << std::endl;
		std::cout << std::endl;
//	}
#endif

	return f_alpha;

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetNormalToAlphaDirection(const ignition::math::Pose3d &_actor_pose) {

	// when speed is 0 then there is no way of calculating the angle THETA_alpha_beta (0 vector length)
	// better way is calculating normal based on actor's yaw than velocity vector

//	if ( print_info ) {
		// std::cout << "BEFORE -- n_alpha: " << *_n_alpha << "  _actor_vel: " << _actor_vel << std::endl;
//	}

	// ignition::math::Vector3d rpy = _actor_pose.Rot().Euler();


	/* all calculations here are based on world coordinate system data -
	 * n_alpha from actor's coordinate system is projected onto
	 * world's coordinate system axes */
	ignition::math::Angle yaw_norm(this->GetYawFromPose(_actor_pose));
	yaw_norm.Normalize();

	// check parameter setting - n_alpha issue there
	switch (param_description_) {

	case(PARAMETER_DESCRIPTION_2011):
			// vector pointing opposite direction
			yaw_norm -= yaw_norm.Pi;
			yaw_norm.Normalize();
			break;

	case(PARAMETER_DESCRIPTION_2014):
	default:
			// do not rotate
			break;

	}

//#ifdef N_ALPHA_V2011
//	yaw_norm -= yaw_norm.Pi; // opposite pointing
//	yaw_norm.Normalize();
//#endif

	ignition::math::Vector3d n_alpha;

	// rotate the vector
	n_alpha.X( +sin(yaw_norm.Radian()) );
	n_alpha.Y( -cos(yaw_norm.Radian()) ); 	// sine and cosine relation the same as in GetNewPose()
	n_alpha.Z(0.0); 						// in-plane movement only at the moment
	n_alpha.Normalize();

	if ( print_info ) {
		std::cout << "\t n_alpha: " << n_alpha << "\t";
	}

	return n_alpha;

}


#ifdef DEBUG_TF

	/*
	// actor's pose
	actor_current_pose = _actor_pose; // current = before update
	double yaw = atan2(_actor_pose.Pos().Y(), _actor_pose.Pos().X());
	actor_current_pose.Set(_actor_pose.Pos(), ignition::math::Vector3d(0.0, 0.0, yaw));

	tf_stamped.header.frame_id = "map";
	tf_stamped.child_frame_id = "actor1_base";

	tf_stamped.transform.translation.x = actor_current_pose.Pos().X();
	tf_stamped.transform.translation.y = actor_current_pose.Pos().Y();
	tf_stamped.transform.translation.z = actor_current_pose.Pos().Z();

	tf_stamped.transform.rotation.x = actor_current_pose.Rot().X();
	tf_stamped.transform.rotation.y = actor_current_pose.Rot().Y();
	tf_stamped.transform.rotation.z = actor_current_pose.Rot().Z();
	tf_stamped.transform.rotation.w = actor_current_pose.Rot().W();

	tf_broadcaster.sendTransform(tf_stamped);

	// actor's normal
	ignition::math::Vector3d rpy_actor = actor_current_pose.Rot().Euler();
	double yaw_normal = atan2(_n_alpha->Y(), _n_alpha->X());
	rpy_actor.Z(yaw_normal);

	tf2::Matrix3x3 quat;
	tf2::Quaternion tf_quat;
	tf_quat.setX(actor_current_pose.Rot().X());
	tf_quat.setY(actor_current_pose.Rot().Y());
	tf_quat.setZ(actor_current_pose.Rot().Z());
	tf_quat.setW(actor_current_pose.Rot().W());

	quat.setRotation(tf_quat);
	double roll, pitch, yawq;
	quat.getRPY(roll, pitch,yawq);

	//std::cout << "yaw: " << rpy_actor.X() << "  yaw_n: " << yaw_normal << std::endl;
	//std::cout << "yaw: " << yawq << "  yaw_n: " << yaw_normal << std::endl;

	actor_current_pose.Set(_actor_pose.Pos(), ignition::math::Vector3d(0.0, 0.0, yaw_normal));

	tf_stamped.header.frame_id = "map";
	tf_stamped.child_frame_id = "actor1_normal";

	tf_stamped.transform.rotation.x = actor_current_pose.Rot().X();
	tf_stamped.transform.rotation.y = actor_current_pose.Rot().Y();
	tf_stamped.transform.rotation.z = actor_current_pose.Rot().Z();
	tf_stamped.transform.rotation.w = actor_current_pose.Rot().W();

	tf_broadcaster.sendTransform(tf_stamped);
	*/

#endif



// ------------------------------------------------------------------- //

/// \return F_alpha_beta - repulsive force created by `beta` object
/// \return d_alpha_beta - distance vector pointing from `alpha` to `beta`
/// \return theta_alpha_beta - angle
ignition::math::Vector3d SocialForceModel::GetInteractionComponent(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_vel,
		// const ignition::math::Pose3d &_actor_target,
		// const ignition::math::Vector3d &_n_alpha,
		// const SFMObjectType &_object_type,
		const ignition::math::Pose3d &_object_pose,
		const ignition::math::Vector3d &_object_vel,
		//const ignition::math::Vector3d &_object_closest_point,
		//const ignition::math::Box &_object_bb)
		const bool &_is_actor
)
{

	// models' closest points already passed to this function - each bounding type
	// already taken into consideration -
	// vector between objects positions
	ignition::math::Vector3d d_alpha_beta = _object_pose.Pos() - _actor_pose.Pos();


#ifdef DEBUG_OSCILLATIONS
	if ( SfmDebugGetCurrentObjectName() == "table1" && SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\n\n\t +++++++ ACTOR pos: " << _actor_pose.Pos() << "\tOBJECT pos: " << _object_closest_point << std::endl;
	}
#endif


	// TODO: adjust Z according to stance?
	d_alpha_beta.Z(0.0); // it is assumed that all objects are in the actor's plane

	// actor's normal (based on velocity vector)
	ignition::math::Vector3d n_alpha = this->GetNormalToAlphaDirection(_actor_pose);

	// DEPRECATED FUNCTION
//	// if the object is not considered as a point - then perform some calculations
//	ignition::math::Vector3d f_alpha_beta = this->GetObjectsInteractionForce(_actor_pose,
//													_actor_vel, _object_pose, _object_vel,
//													n_alpha, d_alpha_beta, // _closest_point,
//													_is_actor);



	// ================================================================================
	// section from "GetObjectsInteractionForce()" function which is DEPRECATED now
	//
	//
#ifdef DEBUG_INTERACTION_FORCE
	if ( print_info ) {
		std::cout << "GetObjectsInteractionForce(): ";
	}
#endif

	// TODO: only 6 closest actors taken into consideration?
	ignition::math::Vector3d f_alpha_beta(0.0, 0.0, 0.0);

	// check length to other object (beta)
	if ( d_alpha_beta.Length() > 10.0 ) {

		// TODO: if no objects nearby the threshold should be increased
		#ifdef DEBUG_INTERACTION_FORCE
		if ( print_info ) {
			std::cout << "\t OBJECT TOO FAR AWAY, ZEROING FORCE! \t d_alpha_beta_length: " << _d_alpha_beta.Length();
			std::cout << std::endl;
		}
		#endif
		return (f_alpha_beta);

	}

#ifdef DEBUG_SHORT_DISTANCE
	if ( !print_info && curr_actor == 0 && d_alpha_beta_length < 0.4 ) {
		print_info = true;
	}
#endif

	/* yaw of an actor is always updated in new pose calculation procedure, so setting the yaw
	 * based on world info should work - actor is always oriented in his movement direction
	 * (if linear speed is non-zero) */
	ignition::math::Angle actor_yaw(GetYawFromPose(_actor_pose));
	actor_yaw.Normalize();

	// OBJECT_YAW used when V2011 and V2014 not #defined
	/* this is a simplified version - object's yaw could be taken from world's info indeed,
	 * but the object's coordinate system orientation is not known, so velocity calculation
	 * still may need to be performed (if needed depending on θ αβ calculation method);
	 * when `_is_actor` flag is set no further velocity calculation needed! */
	ignition::math::Angle object_yaw(GetYawFromPose(_object_pose));
	object_yaw.Normalize();

	RelativeLocation beta_rel_location = LOCATION_UNSPECIFIED;
	double beta_angle_rel = 0.0;
	std::tie(beta_rel_location, beta_angle_rel) = this->GetBetaRelativeLocation(actor_yaw, d_alpha_beta);

	/* total force factor is used to make objects'
	 * that are behind actor interactions weaker */
	double total_force_factor = 1.00;

	/* check whether beta is within the field of view
	 * to determine proper factor for force in case
	 * beta is behind alpha */
	if ( this->IsOutOfFOV(beta_angle_rel) ) {

		// exp function used: e^(-0.5*x)
		total_force_factor = std::exp( -0.5 * d_alpha_beta.Length() );
		#ifdef DEBUG_FORCE_EACH_OBJECT
		if ( SfmGetPrintData() ) {
			#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
			std::cout << SfmDebugGetCurrentActorName();
			#endif
			std::cout << "\nDYNAMIC OBSTACLE (*) --- OUT OF FOV!" << std::endl;
			std::cout << "\t" << SfmDebugGetCurrentObjectName() << " is BEHIND, dist: " << d_alpha_beta.Length() << ",   force will be multiplied by: " << total_force_factor << std::endl;
		}
		#endif

	}


	double v_rel = GetRelativeSpeed(_actor_vel, _object_vel);

	if ( v_rel < 1e-06 ) {

		#ifdef DEBUG_INTERACTION_FORCE
		if ( print_info ) {
			std::cout << "  v_rel = 0, ZEROING FORCE!";
			std::cout << std::endl;
		}
		#endif

		return f_alpha_beta;

	}

	/* angle between velocities of alpha and beta is needed for Fuzzifier */
	double velocities_angle = computeThetaAlphaBetaAngle(_actor_vel, actor_yaw, _object_vel, object_yaw, _is_actor);

	// store angle between objects' (in most cases) velocities
	double theta_alpha_beta = 0.0;

	// check parameter value - theta_alpha_beta issue there
	switch (param_description_) {

	case(PARAMETER_DESCRIPTION_2011):
			//theta_alpha_beta = computeThetaAlphaBetaAngle(_actor_vel, actor_yaw, _object_vel, object_yaw, _is_actor);
			theta_alpha_beta = velocities_angle;
			break;

	case(PARAMETER_DESCRIPTION_2014):
			theta_alpha_beta = computeThetaAlphaBetaAngle(n_alpha, d_alpha_beta);
			break;

	case(PARAMETER_DESCRIPTION_UNKNOWN):
	default:
			theta_alpha_beta = computeThetaAlphaBetaAngle(_actor_pose, &actor_yaw, _object_pose, &object_yaw);
			break;

	}

	ignition::math::Vector3d p_alpha = GetPerpendicularToNormal(n_alpha, beta_rel_location); 	// actor's perpendicular (based on velocity vector)
	double exp_normal = ( (-Bn * theta_alpha_beta * theta_alpha_beta) / v_rel ) - Cn * d_alpha_beta.Length();
	double exp_perpendicular = ( (-Bp * std::fabs(theta_alpha_beta) ) / v_rel ) - Cp * d_alpha_beta.Length();
	f_alpha_beta = n_alpha * An * exp(exp_normal) + p_alpha * Ap * exp(exp_perpendicular);

	// weaken the interaction force when beta is behind alpha
	f_alpha_beta *= total_force_factor;

#ifdef DEBUG_FORCE_EACH_OBJECT

	if ( SfmGetPrintData() ) {
	#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	std::cout << "\n-----------------------\n" << SfmDebugGetCurrentActorName();
	#else
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
	#endif
		std::cout << "\nDYNAMIC OBSTACLE" << std::endl;
		std::cout << "\t" << SfmDebugGetCurrentObjectName() << ": ";
		std::cout << "\tv_rel: " << v_rel << std::endl;
		std::cout << "\tn_alpha: " << n_alpha;
		std::cout << "\texp_n: " << exp_normal << "\ttheta_alpha_beta: " << theta_alpha_beta << "\td_alpha_beta_len: " << d_alpha_beta.Length() << std::endl;
		std::cout << "\tp_alpha: " << p_alpha;
		std::cout << "\texp_p: " << exp_perpendicular;

		std::string location_str;
		if ( beta_rel_location == LOCATION_FRONT ) {
			location_str = "FRONT";
		} else if ( beta_rel_location == LOCATION_BEHIND ) {
			location_str = "BEHIND";
		} else if ( beta_rel_location == LOCATION_RIGHT ) {
			location_str = "RIGHT SIDE";
		} else if ( beta_rel_location == LOCATION_LEFT ) {
			location_str = "LEFT SIDE";
		} else if ( beta_rel_location == LOCATION_UNSPECIFIED ) {
			location_str = "UNKNOWN";
		}
		std::cout << "\t\trel_location: " << location_str << std::endl;

		std::cout << "\tf_alpha_beta: " << f_alpha_beta * interaction_force_factor << "\t\tvec len: " << interaction_force_factor * f_alpha_beta.Length() << std::endl;
	#ifndef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	}
	#endif
	}
#endif



#ifdef DEBUG_INTERACTION_FORCE
	if ( print_info ) {
		std::cout << "Interaction";
		std::cout << "\tv_rel: " << v_rel;
		std::cout << "\texp_n: " << exp_normal;
		std::cout << "\texp_p: " << exp_perpendicular;
		std::cout << "\tf_alpha_beta: " << f_alpha_beta;
		std::cout << std::endl;
	}
#endif


	return (f_alpha_beta);


	/* Algorithm INPUTS are:
	 * - dt
	 * - goal reaching component (acceleration term)
	 * 		- target's pose
	 * 		- current actor's pose
	 * 		- actual speed
	 * 		- desired speed
	 * 		- relaxation time
	 * - dynamic object repulsion component (other people or obstacles)
	 * 		- object's pose
	 * 		- current actor's pose
	 * 		- object's speed
	 * - static object repulsion component (borders)
	 * 		- object's pose
	 * 		- current actor's pose
	 *	- object attraction component (other people or objects)
	 *		- field decrease factor
	 *		- object's pose
	 *		- current actor's pose
	 *
	 */

}

// ------------------------------------------------------------------- //

ignition::math::Angle SocialForceModel::GetYawMovementDirection(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_vel,
		const ignition::math::Vector3d &_sf_vel) {


	// actor's coordinate system transformed into world's coordinate system (SF's)

	// 1st step
	// social force's angle
	ignition::math::Angle yaw_sf( std::atan2(_sf_vel.Y(), _sf_vel.X()) );
	yaw_sf.Normalize();

	// actor's angle transformed to world coordinate system
	double yaw_alpha_w = GetYawFromPose(_actor_pose) - IGN_PI_2;

	// difference used to determine angular rotation direction
	ignition::math::Angle yaw_diff( yaw_sf.Radian() - yaw_alpha_w );
	yaw_diff.Normalize();

	// 2nd step
	/* yaw_increment is a function of speed - the faster actor goes
	 * the less maneuverability he has and less rotations will make	*/
	/// @yaw_increment the less the value is the more reluctant actor
	/// will be to immediate rotations
	/// correlated with parameter @force_min
	double yaw_increment = 0.003 * std::exp( -_actor_vel.Length() ); // 0.009 before - many rotations

	// sign determines angle increment direction
	short int sign = -1;
	( yaw_diff.Radian() >= 0.0 ) ? (sign = +1) : (0);

	// stores the angle increment based on current SF
	double angle_change = 0.0;
	if ( std::fabs(yaw_diff.Radian()) < yaw_increment ) {
		// only small rotation adjustment needed
		angle_change = static_cast<double>(sign) * yaw_diff.Radian();
	} else {
		// truncate big angle change desired by SF
		angle_change = static_cast<double>(sign) * yaw_increment;
	}

	// 3rd step
	// calculate new yaw
	ignition::math::Angle yaw_new( yaw_alpha_w + angle_change );
	yaw_new.Normalize();

	// ----------------------------------------------

	// debug
	#ifdef DEBUG_YAW_MOVEMENT_DIR
	( SfmGetPrintData() ) ? (print_info = true) : (0);
	#endif

	if ( print_info ) {

		#ifdef DEBUG_YAW_MOVEMENT_DIR
		std::cout << "///////////////////////////////////////////////////////////\n";
		std::cout << SfmDebugGetCurrentActorName() << "\tGetYawMovementDirection()" << std::endl;
		std::cout << "\tyaw_alpha: " << GetYawFromPose(_actor_pose) << "\tyaw_alpha_W: " << yaw_alpha_w << "\tyaw_sf: " << yaw_sf.Radian() << "\tsf: " << _sf_vel << std::endl;
		std::cout << "\tyaw_diff NOT norm.: " << (yaw_sf.Radian() - yaw_alpha_w) << "\tyaw_diff_norm: " << yaw_diff.Radian() << std::endl;
		std::cout << "\tangle_change: " << angle_change << "\tyaw_new NOT norm: " << yaw_alpha_w + angle_change << "\tyaw_new norm: " << yaw_new.Radian() << std::endl;
		ignition::math::Vector3d test_vector;
		test_vector.X( cos(yaw_new.Radian()) * _sf_vel.Length() );
		test_vector.Y( sin(yaw_new.Radian()) * _sf_vel.Length() );
		std::cout << "\tEQUAL DIR TEST\tX: " << ((test_vector.X() * _sf_vel.X()) >= 0.0) << "\tY: " << ((test_vector.Y() * _sf_vel.Y()) >= 0.0) << std::endl;
		std::cout << "///////////////////////////////////////////////////////////\n";
		#endif

	}

	#ifdef DEBUG_YAW_MOVEMENT_DIR
	( SfmGetPrintData() ) ? (print_info = false) : (0);
	#endif

	// ----------------------------------------------

	return (yaw_new);

}

// ------------------------------------------------------------------- //

ignition::math::Pose3d SocialForceModel::GetNewPose(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Pose3d &_actor_last_pose,
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Vector3d &_actor_target,
			const ignition::math::Vector3d &_social_force,
			const double &_dt,
			const uint8_t &_stance)
{

	/* II Newton's law equation			a = F / m
	 * Straight-line movement equation 	v = a * t
	 * with the use of 2 above - calculate the resulting ideal velocity caused by social forces */
	ignition::math::Vector3d result_vel = (_social_force / this->mass_person) * _dt;
	ignition::math::Vector3d result_vel_backup = result_vel; // debugging purposes

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
		std::cout << "GetNewPose(): ";
		std::cout << "\tresult_vel: " << result_vel;
	}
#endif

	/* if calculated speed value is bigger than max_speed then perform normalization
	 * leave velocity direction as is, shorten the vector to max possible */
	if ( result_vel.Length() > speed_max ) {

		result_vel = result_vel.Normalize() * speed_max;

		#ifdef DEBUG_NEW_POSE
		if ( print_info ) {
			std::cout << "\t vel TRUNCATED!: " << result_vel;
		}
		#endif

	}

#ifdef DEBUG_NEW_POSE
	ignition::math::Vector3d result_vel_init = result_vel;
	// temporary vector caused by social-force-based displacements
	ignition::math::Vector3d new_position(_actor_pose.Pos().X() + result_vel.X() * _dt,
										  _actor_pose.Pos().Y() + result_vel.Y() * _dt,
										  _actor_pose.Pos().Z());
	if ( print_info ) {
		std::cout << "\nPOSITION1 \torig: " << _actor_pose.Pos() << "\tdelta_x: " << result_vel.X() * _dt << "\tdelta_y: " << result_vel.Y() * _dt << "\tnew_position: " << new_position << std::endl;
	}
#endif

	// -------------------------------------------------------------------------------------------------------------------

	/* Consider the yaw angle of the actor - it is crucial to make him face his
	 * current movement direction / current target.
	 * yaw_new is calculated according to the vector of resulting velocity -
	 * which is based on social force */
	ignition::math::Angle yaw_new = this->GetYawMovementDirection(_actor_pose, _actor_vel, result_vel);

	/* recalculation pros:
	 *  	o smooth rotational motion,
	 *  	o prevents getting stuck in 1 place (observed few times),
	 *  	o prevents sliding (usually caused by rise of v_rel between actors
	 *  	  which causes big interaction)
	 * cons:
	 * 		o prevents immediate action when actor is moving toward an obstacle.
	 * recalculation acts as a inertia force here */

	/* calculate velocity components according to the yaw_new (that was
	 * likely truncated to prevent jumps in rotational movement -
	 * this provides smoothed rotation)
	 * only on-plane motions are supported, thus X and Y calculations */

	/* result_vel vector is projected onto world's coordinate system axes
	 * according to yaw_new (expresses actor's direction in actor's system);
	 * because actor's coord. system is rotated (-90 deg) the projection onto
	 * 	o	x axis: cos(yaw_new - 90 deg) * Len = +sin(yaw_new) * Len
	 * 	o	y axis: sin(yaw_new - 90 deg) * Len = -cos(yaw_new) * Len  */
	// V1
//	result_vel.X( +sin(yaw_new.Radian()) * result_vel.Length() );
//	result_vel.Y( -cos(yaw_new.Radian()) * result_vel.Length() );


	// ------------- V2
	result_vel.X( cos(yaw_new.Radian()) * result_vel.Length() );
	result_vel.Y( sin(yaw_new.Radian()) * result_vel.Length() );

	/* HACK to not mess with angles but to check behavior like maths is right here */
//	if ( result_vel.X() * _social_force.X() < 0.0 ) {
//		// different signs
//		result_vel.X( result_vel.X()*(-1.00) );
//	}
//
//	if ( result_vel.Y() * _social_force.Y() < 0.0 ) {
//		// different signs
//		result_vel.Y( result_vel.Y()*(-1.00) );
//	}

//	result_vel *= 3.00; // to test behavior without recalculation

	if ( print_info ) {
		std::cout << "\n\tSMOOTHING ROTATION - RECALCULATED VEL\tdelta_x: " << result_vel.X() * _dt << "\tdelta_y: " << result_vel.Y() * _dt << '\n' << std::endl;
	}

	/* calculate new pose - consider current pose, velocity and delta of time -
	 * and set the new pose component values (for position and orientation) */

	// TODO: fix forced pose.Z() and pose.Roll() according to current 'stance'
	// TODO: hard-coded value for STANDING stance
	// TODO: assuming standing pose thus roll angle is set to half-pi (STANDING)

	ignition::math::Pose3d new_pose;
	new_pose.Set(_actor_pose.Pos().X() + result_vel.X() * _dt,
				 _actor_pose.Pos().Y() + result_vel.Y() * _dt,
				 1.2138,
				 (IGN_PI/2),
				 0,
				 yaw_new.Radian() + IGN_PI_2); 	// V2 // transform back to actor's CS
				 // yaw_new.Radian()); 			// V1


#ifdef DEBUG_NEW_POSE

	#ifdef DEBUG_JUMPING_POSITION
	if ( (new_pose.Pos() - _actor_pose.Pos()).Length() > 0.1 ) {
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n";
		std::cout << "\nJUMP_IN_POSITION\tinit_pose: " << _actor_pose << "\tresult_vel init: " << result_vel_init << std::endl;
		std::cout << "\t\tnew_pose: " << new_pose << "result_vel recalculated: " << result_vel << std::endl;
		std::cout << "\t\tactor_ID: " << curr_actor << std::endl;
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n";
	}
	#endif

	if ( print_info ) {
		std::cout << "\nPOSITION2 \torig: " << _actor_pose.Pos() << "\tdelta_x: " << result_vel.X() * _dt << "\tdelta_y: " << result_vel.Y() * _dt << "\tnew_position: " << new_pose.Pos(); // << std::endl;
		std::cout << std::endl;
		std::cout << "---------------------------------------------------------------------------------";
		std::cout << std::endl;
	}

#endif

#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	if ( SfmGetPrintData() ) {
	std::cout <<  SfmDebugGetCurrentActorName() << " | GetNewPose() CALCULATION\n";
	std::cout << "\tactor_yaw_init: " << (_actor_pose.Rot().Yaw()-IGN_PI_2) << "\tactor_yaw_mod: " << yaw_new.Radian() << std::endl;
	std::cout << "\tresult_vel_angle: " << std::atan2(result_vel.Y(), result_vel.X()) << std::endl;
	std::cout << "\t\tsf: " << _social_force << "\tresult_vel_init: " << result_vel_backup << "\tresult_vel_mod: " << result_vel;
	if ( result_vel.X() > 0.0 && result_vel.Y() > 0.0 ) {
		std::cout << "\tI QUARTER";
	} else if ( result_vel.X() > 0.0 && result_vel.Y() < 0.0 ) {
		std::cout << "\tIV QUARTER";
	} else if ( result_vel.X() < 0.0 && result_vel.Y() > 0.0 ) {
		std::cout << "\tII QUARTER";
	} else if ( result_vel.X() < 0.0 && result_vel.Y() < 0.0 ) {
		std::cout << "\tIII QUARTER";
	}
	std::cout << "\nPOSITION \torig: " << _actor_pose.Pos() << "\tdelta_x: " << result_vel.X() * _dt << "\tdelta_y: " << result_vel.Y() * _dt << "\tnew_position: " << new_pose.Pos(); // << std::endl;
	std::cout << std::endl;
	std::cout << "---------------------------------------------------------------------------------\n";
	}
#endif

	return new_pose;

}

// ------------------------------------------------------------------- //

// TODO: DEPRECATED?
bool SocialForceModel::IsActorFacingTheTarget(	const ignition::math::Angle _yaw,
												const ignition::math::Vector3d _target)
{

	ignition::math::Angle yaw_target( std::atan2( _target.X(), _target.Y() ) );
	yaw_target.Normalize();

	ignition::math::Angle yaw_result( yaw_target.Radian() - _yaw.Radian() );
	yaw_result.Normalize();

#ifdef DEBUG_ACTOR_FACING_TARGET
	if ( print_info ) {
		std::cout << "\n\tIsActorFacingTheTarget()\tyaw_actor: " << _yaw.Radian() << "\tyaw_target: " << yaw_target.Radian() << "\tyaw_result: " << yaw_result.Radian();
	}
#endif

	/* The threshold angle is taken - 0-(TH) degs should be considered as well as (180-TH)-180 */
	const unsigned short int THRESHOLD_ANGLE = 25;
	if ( 	std::fabs(yaw_result.Radian()) <= IGN_DTOR(THRESHOLD_ANGLE) ||
			std::fabs(yaw_result.Radian()) >= IGN_DTOR(180-THRESHOLD_ANGLE) )
	{
		return true;
	} else {
		return false;
	}


}

// ------------------------------------------------------------------- //

void SocialForceModel::SetParameterValues() {

	/* Invoking this function to each actor will create a population in which there are
	 * everyone moving in a slightly other way */

	std::default_random_engine rand_gen;	// random number generator

	// desired speed (based on (Moussaid et al. (2009))
	std::normal_distribution<float> dist_spd_desired(1.29F, 0.19F);
	speed_desired = dist_spd_desired(rand_gen);

	// relaxation time (based on (Moussaid et al. (2009))
	std::normal_distribution<float> dist_tau(0.54F, 0.05F);
	relaxation_time = dist_tau(rand_gen);

	// ----------------------------- Model C ------------------------------------------------------ //
	// Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014)
	// Generate random value of mean a and standard deviation b
	std::normal_distribution<float> dist_an(0.2615F, 0.0551F);		An = dist_an(rand_gen);
	std::normal_distribution<float> dist_bn(0.4026F, 0.1238F);		Bn = dist_bn(rand_gen);
	std::normal_distribution<float> dist_cn(2.1614F, 0.3728F);		Cn = dist_cn(rand_gen);
	std::normal_distribution<float> dist_ap(1.5375F, 0.3084F);		Ap = dist_ap(rand_gen);
	std::normal_distribution<float> dist_bp(0.4938F, 0.1041F);		Bp = dist_bp(rand_gen);
	std::normal_distribution<float> dist_cp(0.5710F, 0.1409F);		Cp = dist_cp(rand_gen);
	std::normal_distribution<float> dist_aw(0.3280F, 0.1481F);		Aw = dist_aw(rand_gen);
	std::normal_distribution<float> dist_bw(0.1871F, 0.0563F);		Bw = dist_bw(rand_gen);

#ifdef DEBUG_SFM_PARAMETERS
	std::cout << "\t speed_desired: " << speed_desired << std::endl;
	std::cout << "\t relaxation_time: " << relaxation_time << std::endl;
	std::cout << "\t An: " << An << std::endl;
	std::cout << "\t Bn: " << Bn << std::endl;
	std::cout << "\t Cn: " << Cn << std::endl;
	std::cout << "\t Ap: " << Ap << std::endl;
	std::cout << "\t Bp: " << Bp << std::endl;
	std::cout << "\t Cp: " << Cp << std::endl;
	std::cout << "\t Aw: " << Aw << std::endl;
	std::cout << "\t Bw: " << Bw << std::endl;
	std::cout << std::endl;
#endif

}

// ------------------------------------------------------------------- //

double SocialForceModel::GetRelativeSpeed(
		const ignition::math::Vector3d &_actor_velocity,
		const ignition::math::Vector3d &_object_velocity) {

#ifdef DEBUG_REL_SPEED

	ignition::math::Vector3d rel_vel = _object_velocity - _actor_velocity;
	if ( print_info ) {
		std::cout << std::endl;
		std::cout << "GetRelativeSpeed(): ";
		std::cout << "  obj_vel: " << _object_velocity;
		std::cout << "  act_vel: " << _actor_velocity;
		std::cout << "  v_rel:  " << rel_vel;
		std::cout << "  spd_rel: " << rel_vel.Length();
		std::cout << std::endl;
	}

	return rel_vel.Length();

#else

	return ( (_object_velocity - _actor_velocity).Length() );

#endif

}

// ============================================================================

inline double SocialForceModel::GetYawFromPose(const ignition::math::Pose3d &_pose) {

	// the actor's offset yaw is considered
	// return (_actor_pose.Rot().Yaw() + (IGN_PI / 2));

	// when offset already considered
	return (_pose.Rot().Yaw());
}

// ============================================================================

//ignition::math::Vector3d SocialForceModel::GetObjectsInteractionForce(
//		const ignition::math::Pose3d &_actor_pose,
//		const ignition::math::Vector3d &_actor_velocity,
//		const ignition::math::Pose3d &_object_pose,
//		const ignition::math::Vector3d &_object_velocity,
//		const ignition::math::Vector3d &_n_alpha, 		// actor's normal (based on velocity vector)
//		const ignition::math::Vector3d &_d_alpha_beta, 	// vector between objects positions
//		//const ignition::math::Vector3d &_closest_point,
//		const bool &_is_actor
//)
//{
//
//	// FUNCTION DEPRECATED
//	ignition::math::Vector3d aaa;
//	return ( aaa );
//
//}

// ------------------------------------------------------------------- //



// ***************************************************************************
// ***************************************************************************
// one of 3 possibilities of calculating theta_alpha_beta will be chosen
// ***************************************************************************
// ***************************************************************************



// #if !defined(THETA_ALPHA_BETA_V2011) && !defined(THETA_ALPHA_BETA_V2014)

// dynamic objects interaction
double SocialForceModel::computeThetaAlphaBetaAngle(
		const ignition::math::Pose3d &_actor_pose,
		ignition::math::Angle *_actor_yaw,
		const ignition::math::Pose3d &_object_pose,
		ignition::math::Angle *_object_yaw)
{
    /*
	 *
	 * NOTE: below method (very simple and naive) of calculating the angle is correct
	 * only when both objects are:
	 * 		o dynamic,
	 * 		o currently moving,
	 * 		o already aligned with the to-target-direction,
	 * 		o there are no obstacles in the environment that will make the object not move along a straight line.
	 */

	// only on-plane movement considered
#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "GetAngleBetweenObjectsVelocities(): ";
	}
#endif

	ignition::math::Angle yaw_diff(_actor_yaw->Radian() - _object_yaw->Radian());
	yaw_diff.Normalize();

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "\t yaw_actor: " << _actor_yaw->Radian() << "  yaw_object: " << _object_yaw->Radian() << "  yaw_diff: " << yaw_diff.Radian() << std::endl;
	}
#endif

	return ( yaw_diff.Radian() );

}

// #endif

// ------------------------------------------------------------------- //

//#if defined(THETA_ALPHA_BETA_V2011)

// 2011 - "θ αβ - angle between velocity of pedestrian α and the displacement of pedestrian β"
double SocialForceModel::computeThetaAlphaBetaAngle(
		const ignition::math::Vector3d &_actor_vel,
		const ignition::math::Angle &_actor_yaw,
		const ignition::math::Vector3d &_object_vel,
		const ignition::math::Angle &_object_yaw,
		const bool &_is_actor)
{

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "GetAngleBetweenObjectsVelocities(): ";
	}
#endif


	/* Actor's info is based on his world's pose - it is easier to treat the current actor
	 * as an object that always keeps aligned to his movement direction.
	 * On the other hand the other object's orientation is not known and its velocity
	 * will be used to determine it's yaw. */

	/* The problem is that actor's linear velocity couldn't be set so it can't be visible
	 * from the world_ptr (always 0, no matter what was set, same in model_ptr) - so when
	 * calculating interaction between 2 actors the other one (so-called beta) has a velocity
	 * linear velocity of (0,0,0). */

	ignition::math::Angle yaw_diff;

	// check if the other object is an actor
	if ( _is_actor ) {

		#ifdef DEBUG_GEOMETRY_1
		if ( print_info ) {
			std::cout << "\t++another ACTOR++";
		}
		#endif

		/* both actors coordinate systems are oriented the same
		 * so no extra calculations need to be performed (relative angle) */

		/* this is a `lightweight` version - another one is based on dot product
		 * and arccos calculation (used in the dynamic object case) - result would
		 * be equal here (when both actors are moving);
		 * with a use of this version there is an ability to calculate the angle
		 * even when one of the actors is currently only rotating */
		yaw_diff.Radian((_actor_yaw.Radian() - _object_yaw.Radian()));
		yaw_diff.Normalize();

	} else {

		/* so first let's check if the considered object is static or dynamic;
		 * if it is static then the only thing that could be done is calculating
		 * the yaw_diff as in `_is_actor` case */
		if ( _object_vel.Length() < 1e-06 ) {

			#ifdef DEBUG_GEOMETRY_1
			if ( print_info ) {
				std::cout << "\t++STATIC object++"; // THIS SHOULD NOT HAPPEN AFTER ADDING COMPONENT FOR STATIC OBSTACLES!
			}
			#endif

			// yaw_diff.Radian((_actor_yaw.Radian() - _object_yaw.Radian()));
			// transform actor's yaw to the world's coordinate system
			yaw_diff.Radian((_actor_yaw.Radian() - IGN_PI_2 - _object_yaw.Radian()));
			yaw_diff.Normalize();

		} else {

			#ifdef DEBUG_GEOMETRY_1
			if ( print_info ) {
				std::cout << "\t++!!! DYNAMIC object !!!++";
			}
			#endif

//			 TODO: debug this, NOT TESTED!
//			 velocities are expressed in world's coordinate system
//
//			 V1
//			 transform object's yaw to actor's coordinate system by adding 90 deg
//			ignition::math::Angle yaw_temp( std::atan2( _object_vel.Y(), _object_vel.X() ) + (IGN_PI/2) );
//			yaw_temp.Normalize();
//
//			yaw_diff.Radian( _actor_yaw.Radian() - yaw_temp.Radian() );

			// V2 - OK
			// both velocities are expressed in world's coordinate system
			// formula -> Section "Examples of spatial tasks" @ https://onlinemschool.com/math/library/vector/angl/
			yaw_diff.Radian( std::acos( _actor_vel.Dot(_object_vel) / (_actor_vel.Length() * _object_vel.Length()) ) );
			yaw_diff.Normalize();

		}

	}


#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "\t yaw_actor: " << _actor_yaw.Radian() << "  yaw_object: " << _object_yaw.Radian() << "  yaw_diff: " << yaw_diff.Radian() << "\tactor_vel: " << _actor_vel << "\tobj_vel: " << _object_vel << std::endl;
	}
#endif

	return (yaw_diff.Radian());

}

//#endif

// ------------------------------------------------------------------- //

//#if defined(THETA_ALPHA_BETA_V2014)

// 2014 - "φ αβ is the angle between n α and d αβ"
double SocialForceModel::computeThetaAlphaBetaAngle(
			const ignition::math::Vector3d &_n_alpha, 		// actor's normal (based on velocity vector)
			const ignition::math::Vector3d &_d_alpha_beta  	// vector between objects positions
	)

{

	/* both n α and d αβ are expressed in world's coordinate system so
	 * simple angle difference should do the job */

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "GetAngleBetweenObjectsVelocities(): ";
	}
#endif

	ignition::math::Angle angle_n_alpha( std::atan2( _n_alpha.Y(), _n_alpha.X() ) );
	angle_n_alpha.Normalize();

	ignition::math::Angle angle_d_alpha_beta( std::atan2( _d_alpha_beta.Y(), _d_alpha_beta.X() ) );
	angle_d_alpha_beta.Normalize();

	ignition::math::Angle phi_alpha_beta(angle_n_alpha.Radian() - angle_d_alpha_beta.Radian());
	phi_alpha_beta.Normalize();

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "\tangle_n_alpha: " << angle_n_alpha.Radian() << "\tangle_d_alpha_beta: " << angle_d_alpha_beta.Radian() << "\tdiff: " << phi_alpha_beta.Radian() << std::endl;
	}
#endif

	return (phi_alpha_beta.Radian());

}

//#endif

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetPerpendicularToNormal(

		const ignition::math::Vector3d &_n_alpha,
		const uint8_t &_beta_rel_location)

{

	/* Depending on which side beta is on relative to n_alpha, the p_alpha (perpendicular vector)
	 * will point to direction opposite to the side where beta is */

	/*
	ignition::math::Vector3d p_alpha;
	if ( _beta_rel_location == LOCATION_RIGHT ) {
		p_alpha = _n_alpha.Perpendicular();
		// return (_n_alpha.Perpendicular());
	} else if ( _beta_rel_location == LOCATION_LEFT ) {

		// ignition::math::Vector3d p_alpha;

		// inverse-perpendicular vector calculations based on ignition library
		static const double sqr_zero = 1e-06 * 1e-06;
		ignition::math::Vector3d to_cross = {0, 0, 1}; // TODO: -1?
		p_alpha = _n_alpha.Cross(to_cross);

		// Check the length of the vector
		if (p_alpha.SquaredLength() < sqr_zero)
		{

			to_cross = {0, -1, 0};
			p_alpha = p_alpha.Cross(to_cross);
			std::cout << "GetPerpendicularToNormal(): " << "\t TOO SHORT! " << std::endl;

		}
	}
	*/

	ignition::math::Vector3d p_alpha;
	static const double sqr_zero = 1e-06 * 1e-06;
	ignition::math::Vector3d to_cross;

	if ( _beta_rel_location == LOCATION_LEFT ) {

		// check parameter - n_alpha issue there
		switch (param_description_) {

		case(PARAMETER_DESCRIPTION_2011):
				to_cross.Set(0.0, 0.0, -1.0);
				break;

		case(PARAMETER_DESCRIPTION_2014):
		default:
				to_cross.Set(0.0, 0.0,  1.0);
				break;

		}

//		#if 	defined(N_ALPHA_V2011)
//		to_cross.Set(0.0, 0.0, -1.0);
//		#elif 	defined(N_ALPHA_V2014)
//		to_cross.Set(0.0, 0.0,  1.0);
//		#endif

	} else if ( _beta_rel_location == LOCATION_RIGHT ) {

		// check parameter - n_alpha issue there
		switch (param_description_) {

		case(PARAMETER_DESCRIPTION_2011):
				to_cross.Set(0.0, 0.0,  1.0);
				break;
		case(PARAMETER_DESCRIPTION_2014):
		default:
				to_cross.Set(0.0, 0.0, -1.0);
				break;

		}

//		#if 	defined(N_ALPHA_V2011)
//		to_cross.Set(0.0, 0.0,  1.0);
//		#elif 	defined(N_ALPHA_V2014)
//		to_cross.Set(0.0, 0.0, -1.0);
//		#endif

	} else {

#ifdef DEBUG_GEOMETRY_1
		if ( print_info ) {
			std::cout << "BEHIND!\t" << std::endl;
		}
#endif

	}

	p_alpha = _n_alpha.Cross(to_cross);

	// Check the length of the vector
	if (p_alpha.SquaredLength() < sqr_zero) {

		// this should not happen
		to_cross = {0, -1, 0};
		p_alpha = p_alpha.Cross(to_cross);
#ifdef DEBUG_GEOMETRY_1
		if ( print_info ) {
			std::cout << "GetPerpendicularToNormal(): " << "\t TOO SHORT! " << std::endl;
		}
#endif

	}

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "GetPerpendicularToNormal(): " << "  x: " << p_alpha.X() << "  y: " << p_alpha.Y() << "  z: " << p_alpha.Z() << std::endl;
	}
#endif

	return p_alpha;

}

// ------------------------------------------------------------------- //

// TODO: make it return a tuple <RelativeLocation rel_loc, double rel_angle> - for detailed info connected with BEHIND objects?
std::tuple<RelativeLocation, double> SocialForceModel::GetBetaRelativeLocation(
		const ignition::math::Angle &_actor_yaw,
		const ignition::math::Vector3d &_d_alpha_beta)
{

	RelativeLocation rel_loc = LOCATION_UNSPECIFIED;
	ignition::math::Angle angle_relative; 		// relative to actor's (alpha) direction
	ignition::math::Angle angle_d_alpha_beta;	// stores yaw of d_alpha_beta

	ignition::math::Vector3d d_alpha_beta_norm = _d_alpha_beta;
	d_alpha_beta_norm.Normalize();

	// when normalized vector used with atan2 then division by euclidean distance not needed
	//angle_d_alpha_beta.Radian( std::atan2(d_alpha_beta_norm.X(), d_alpha_beta_norm.Y()) );	// V1
	angle_d_alpha_beta.Radian( std::atan2(d_alpha_beta_norm.Y(), d_alpha_beta_norm.X()) ); 	// V2
	angle_d_alpha_beta.Normalize();

	//angle_relative = _actor_yaw + angle_d_alpha_beta;	// V1

	// V2
	//angle_relative = _actor_yaw - angle_d_alpha_beta; 	// actor's coordinate system rotation considered (relative to world coord. sys.)

	/* V2 NOTES:
	 *
	 */
	angle_relative = _actor_yaw - angle_d_alpha_beta;

	// V3
	// corrected in terms of transforming to world's coordinate system
	ignition::math::Angle actor_yaw_corrected(_actor_yaw.Radian() - IGN_PI_2);
	actor_yaw_corrected.Normalize(); // V4
	angle_relative.Radian( angle_d_alpha_beta.Radian() - actor_yaw_corrected() );
	angle_relative.Normalize(); // V4

	// angle_relative.Normalize(); // V1

#ifdef DEBUG_GEOMETRY_2
	if ( print_info ) {
		std::cout << "\n GetBetaRelativeLocation() " << "ACTOR yaw: " << _actor_yaw.Radian() << "  ANGLE d_alpha_beta: " << angle_d_alpha_beta.Radian() << "  ANGLE sum: " << angle_relative.Radian();
	}
	std::string txt_dbg;
#endif

	// LOCATION_FRONT ~ hysteresis regulator
	/*
	if ( angle_relative.Radian() >= -IGN_DTOR(9) &&
		 angle_relative.Radian() <= +IGN_DTOR(9) ) {
	*/

	/* SFM FRONT or SFM_BACK to be specific - added exponentially decreasing
	 * interaction for objects that are behind so relative angle calculations
	 * extended with close to Pi value case */
	if ( std::fabs(angle_relative.Radian()) <= IGN_DTOR(9) ||
		 std::fabs(angle_relative.Radian()) >= (IGN_PI - IGN_DTOR(9)) ) {

		rel_loc = LOCATION_FRONT;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "FRONT";
#endif
	/* // LOCATION_BEHIND DEPRECATED HERE
	} else if ( IsOutOfFOV(angle_relative.Radian() ) ) { // consider FOV

		rel_loc = LOCATION_BEHIND;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "BEHIND";
#endif
	*/
	} else if ( angle_relative.Radian() <= 0.0 ) { // 0.0 ) {

		rel_loc = LOCATION_RIGHT;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "RIGHT";
#endif

	} else if ( angle_relative.Radian() > 0.0 ) { // 0.0 ) {

		rel_loc = LOCATION_LEFT;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "LEFT";
#endif

	}

#ifdef DEBUG_GEOMETRY_2
	if ( print_info ) {
		std::cout << "  " << txt_dbg << std::endl;
	}
#endif

	/* historical data considered when calculating relative location - it is crucial to not
	 * allow minor yaw rotations to switch from left to right etc. thus a relative angle
	 * must exceed certain threshold before it will be in fact switched;
	 * LOCATION_BEHIND must not be found in map_models_rel_locations! */

#ifdef DEBUG_OSCILLATIONS
	if ( SfmDebugGetCurrentObjectName() == "table1" && SfmDebugGetCurrentActorName() == "actor1" ) {
		std::string location_str;
		if ( rel_loc == LOCATION_FRONT ) {
			location_str = "FRONT";
		} else if ( rel_loc == LOCATION_BEHIND ) {
			location_str = "BEHIND";
		} else if ( rel_loc == LOCATION_RIGHT ) {
			location_str = "RIGHT SIDE";
		} else if ( rel_loc == LOCATION_LEFT ) {
			location_str = "LEFT SIDE";
		} else if ( rel_loc == LOCATION_UNSPECIFIED ) {
			location_str = "UNKNOWN";
		}
		std::cout << "\t" << SfmDebugGetCurrentObjectName() << "'s relative location: " << location_str <<  "\tactor: " << SfmDebugGetCurrentActorName();
		std::cout << "\n\t\td_alpha_beta: x " << d_alpha_beta_norm.X() << "  y " << d_alpha_beta_norm.Y() << "\tangle_actor_yaw_RAW: " << _actor_yaw.Radian() << "\tangle_actor_yaw_CORR: " << actor_yaw_corrected.Radian() << "\tangle_d_ab: " << angle_d_alpha_beta.Radian() << /* "\tangle_diff: " << (angle_d_alpha_beta.Radian()-actor_yaw_corrected.Radian())<< */ "\tangle_rel: " << angle_relative.Radian();
	}
#endif

	/* if the angle_relative is above few degrees value then the historical value may be discarded */
	if ( rel_loc != LOCATION_FRONT ) {
		map_models_rel_locations[SfmDebugGetCurrentObjectName()] = rel_loc;
	} else {
#ifdef DEBUG_OSCILLATIONS
		std::cout << "\t:::::::::::::::::::::::::HIST REL ANG!:::::::::::::::::::::::::::::::::::::::::::\t";
#endif
		return ( std::make_tuple(map_models_rel_locations[SfmDebugGetCurrentObjectName()], angle_relative.Radian()) );
	}

	return ( std::make_tuple(rel_loc, angle_relative.Radian()) );

}

// ------------------------------------------------------------------- //

bool SocialForceModel::IsOutOfFOV(const double &_angle_relative) {

	if ( std::fabs(_angle_relative) >= fov ) {
		return true;
	}

	return false;

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetForceFromStaticObstacle(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_velocity,
		const ignition::math::Pose3d &_object_pose,
		const double &_dt)
{

	/* elliptic formulation - `14 article - equations (3) and (4) */

	// distance vector
	ignition::math::Vector3d d_alpha_i = _actor_pose.Pos() - _object_pose.Pos(); // proper direction
	d_alpha_i.Z(0.00); // planar

	// acceleration
	ignition::math::Vector3d y_alpha_i = _actor_velocity * _dt;
	y_alpha_i.Z(0.00); // planar

	// semi-minor axis of the elliptic formulation
	double w_alpha_i = 0.5 * sqrt( std::pow((d_alpha_i.Length() + (d_alpha_i - y_alpha_i).Length()),2) -
								   std::pow(y_alpha_i.Length(), 2) );

	// division by ~0 prevention - returning zeros vector instead of NaNs
	if ( (std::fabs(w_alpha_i) < 1e-08) || (std::isnan(w_alpha_i)) || (d_alpha_i.Length() < 1e-08) ) {

		#ifdef DEBUG_FORCE_EACH_OBJECT
		if ( SfmGetPrintData() ) {
			std::cout << "\n-----------------------\n" << SfmDebugGetCurrentActorName();
			std::cout << "\nSTATIC OBSTACLE ============= ERROR ===================" << std::endl;
			std::cout << "\t" << SfmDebugGetCurrentObjectName() << ": ";
			std::cout << "\td_alpha_i: " << d_alpha_i << " \tlen: " << d_alpha_i.Length() << std::endl;
			std::cout << "\ty_alpha_i: " << y_alpha_i;
			std::cout << "\tw_alpha_i: " << w_alpha_i << std::endl;
			std::cout << "\tFAIL w_alpha_i small: " << (std::fabs(w_alpha_i) < 1e-08) << std::endl;
			std::cout << "\tFAIL w_alpha_i NaN: " << (std::isnan(w_alpha_i)) << std::endl;
			std::cout << "\td_alpha_i Length FAIL: " << (d_alpha_i.Length() < 1e-08) << std::endl;
			std::cout << "\tf_alpha_i: " << ignition::math::Vector3d(0.0, 0.0, 0.0) << std::endl;
		}
		#endif

		return ( ignition::math::Vector3d(0.0, 0.0, 0.0) );
	}

	// ~force (acceleration) calculation
	ignition::math::Vector3d f_alpha_i;
	f_alpha_i = this->Aw * exp(-w_alpha_i/this->Bw) * ((d_alpha_i.Length() + (d_alpha_i - y_alpha_i).Length()) /
			    2*w_alpha_i) * 0.5 * (d_alpha_i.Normalized() + (d_alpha_i - y_alpha_i).Normalized());

	// FIXME: temp mass factor
	// setting too high produces noticeable accelerations around objects
	f_alpha_i *= 30.0;

#ifdef DEBUG_FORCE_EACH_OBJECT
	if ( SfmGetPrintData() ) {
	#ifdef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	std::cout << "\n-----------------------\n" << SfmDebugGetCurrentActorName();
	#else
	if ( SfmDebugGetCurrentActorName() == "actor1" ) { // && SfmDebugGetCurrentObjectName() == "table1") {
	#endif
		std::cout << "\nSTATIC OBSTACLE" << std::endl;
		std::cout << "\t" << SfmDebugGetCurrentObjectName() << ": ";
		std::cout << "\td_alpha_i: " << d_alpha_i << " \tlen: " << d_alpha_i.Length() << std::endl;
		std::cout << "\ty_alpha_i: " << y_alpha_i;
		std::cout << "\tw_alpha_i: " << w_alpha_i << std::endl;
		std::cout << "\tf_alpha_i: " << f_alpha_i * interaction_force_factor << std::endl;
	#ifndef DEBUG_FORCE_PRINTING_SF_TOTAL_AND_NEW_POSE
	}
	#endif
	}
#endif

	return (f_alpha_i);

}

// ------------------------------------------------------------------- //

SocialForceModel::~SocialForceModel() { }

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace sfm */
