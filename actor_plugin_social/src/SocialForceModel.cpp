/*
 * SocialForceModel.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#include <SocialForceModel.h>
#include <cmath>
#include <tgmath.h>		// fabs()

/* Uncommenting hangs debug messages and makes 2 actors move the same way -
 * just like adding the `static` function to ActorPlugin class */
// #define GRID_MAP_VISUALIZATION

#ifdef GRID_MAP_VISUALIZATION

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#endif

// ----------------------------------------

namespace SocialForceModel {

#define SFM_RIGHT_SIDE 0
#define SFM_LEFT_SIDE  1
#define SFM_BEHIND     2

// #define THETA_ALPHA_BETA_CONSIDER_ZERO_VELOCITY
// #define BETA_REL_LOCATION_BASED_ON_NORMAL


// #define DEBUG_SFM_PARAMETERS
#define DEBUG_GEOMETRY_1 // angle correctes etc.
#define DEBUG_GEOMETRY_2 // relative location
#define DEBUG_INTERNAL_ACC
#define DEBUG_INTERACTION_FORCE
#define DEBUG_REL_SPEED
#define DEBUG_NEW_POSE
#define DEBUG_ACTOR_FACING_TARGET
#define DEBUG_BOUNDING_BOX


#ifdef DEBUG_NEW_POSE
#define DEBUG_JUMPING_POSITION
unsigned int curr_actor = 3;				// TODO: couldn't catch an event to debug this
#endif

// ---------------------------------

#define CALCULATE_INTERACTION

// ---------------------------------

#define ACTOR_ID_NOT_FOUND	255u

// - - - - - - - - - - - - - - - - -

#include "print_info.h"
static bool print_info = false;
static int print_counter = 0;

#define NEW_YAW_BASE


// ------------------------------------------------------------------- //

#ifdef GRID_MAP_VISUALIZATION

#define MAP_BOUND_X     3.0f
#define MAP_BOUND_Y     2.0f
#define MAP_RESOLUTION  0.1f

// Create grid map
static grid_map::GridMap gridmap({"layer"});

#endif


// ------------------------------------------------------------------- //

// TODO: add references to arguments to avoid copy

SocialForceModel::SocialForceModel():

		// TODO: note that IT IS REQUIRED TO NAME ALL ACTORS "actor..."

	fov(1.80), speed_max(1.50), yaw_max_delta(20.0 / M_PI * 180.0), mass_person(1),
	desired_force_factor(200.0), interaction_force_factor(5000.0), force_max(2000.0), force_min(750.0) {

	SetParameterValues();

	/* Algorithm PARAMETERS are:
	 * - relaxation time must be given here
	 * - kind of coefficient for attraction artificial potential field (decreases over time)
	 * - FOV
	 * - direction weight
	 * - max speed
	 */

#ifdef GRID_MAP_VISUALIZATION
	gridmap.setFrameId("map");
	gridmap.setGeometry(grid_map::Length(1.0, 1.0), 0.1);
	gridmap["layer"].setRandom();
#endif

}

// ------------------------------------------------------------------- //

void SocialForceModel::Init(const unsigned short int _mass_person, const float _desired_force_factor, const float _interaction_force_factor) {



}

// ------------------------------------------------------------------- //

unsigned int SocialForceModel::GetActorID(const std::string _name, const std::map<std::string, unsigned int> _map) {

	std::map<std::string, unsigned int>::const_iterator it;
	it = _map.find(_name);
	if ( it != _map.end() ) {
//		std::cout << "GetActorID - " << it->first << "\t id: " << it->second << std::endl;
	} else {
		return ACTOR_ID_NOT_FOUND;
	}
	return it->second;

}

// ------------------------------------------------------------------- //

bool SocialForceModel::IsActor(const std::string &_name) {

	std::size_t found_pos = _name.find("actor");
	if ( found_pos != std::string::npos ) {
		return true;
	}
	return false;

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetActorVelocity(const gazebo::physics::ModelPtr &_model_ptr,
		const std::map<std::string, unsigned int>  _map,
		const std::vector<ignition::math::Vector3d> _actors_velocities) {

	unsigned int actor_id = this->GetActorID(_model_ptr->GetName(), _map);
	if ( actor_id == ACTOR_ID_NOT_FOUND ) {
		return ignition::math::Vector3d(0.0, 0.0, 0.0);
	}
	return _actors_velocities[actor_id];

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetSocialForce(
	const gazebo::physics::WorldPtr _world_ptr,
	const std::string _actor_name,
	const ignition::math::Pose3d _actor_pose,
	const ignition::math::Vector3d _actor_velocity,
	const ignition::math::Vector3d _actor_target,
	const std::map<std::string, unsigned int>  _map_actor_name_id,
	const std::vector<ignition::math::Vector3d> _actors_velocities)
{

#ifdef DEBUG_JUMPING_POSITION
	curr_actor = this->GetActorID(_actor_name, _map_actor_name_id);
#endif

//	unsigned int actor_id = this->GetActorID(_actor_name, _map_actor_name_id);
//	if ( actor_id == ACTOR_ID_NOT_FOUND ) {
//		return ignition::math::Vector3d(0.0, 0.0, 0.0);
//	}

	// easier to debug
	if ( _actor_name == "actor1" ) {
		if ( print_counter++ == 250 ) {
			std::cout << " -------------- PRINT_INFO triggered TRUE ------------------ " << std::endl;
			print_info = true;
			print_counter = 0;
		} else {
			print_info = false;
		}
	} else {
		print_info = false;
	}

	// allocate all variables needed in loop
	ignition::math::Vector3d f_alpha = this->GetInternalAcceleration(_actor_pose,
																	 _actor_velocity,
																	 _actor_target);
	ignition::math::Vector3d f_interaction(0.0, 0.0, 0.0);
	ignition::math::Vector3d f_alpha_beta(0.0, 0.0, 0.0);

#ifdef CALCULATE_INTERACTION
	/* model_vel contains model's velocity (world's object or actor) - for the actor this is set differently
	 * it was impossible to set actor's linear velocity by setting it by the model's class method */
	ignition::math::Vector3d model_vel;

	/* */

	/* below flag is used as a workaround for the problem connected with being unable to set actor's
	 * velocity and acceleration in the gazebo::physics::WorldPtr */
	bool is_an_actor = false;
	gazebo::physics::ModelPtr model_ptr;

	for ( unsigned int i = 0; i < _world_ptr->ModelCount(); i++ ) {

		model_ptr = _world_ptr->ModelByIndex(i);

		if ( model_ptr->GetName() == _actor_name ) {
			// do not calculate social force from itself
			continue;
		}

		// test world specific names
		if ( model_ptr->GetName() == "cafe" || model_ptr->GetName() == "ground_plane" ) {
			// do not calculate social force from the objects he is stepping on
			continue;
		}

		if ( print_info ) {
			std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
		}

		if ( (is_an_actor = this->IsActor(model_ptr->GetName())) ) {
			model_vel = GetActorVelocity(model_ptr, _map_actor_name_id, _actors_velocities);
		} else {
			model_vel = model_ptr->WorldLinearVel();
		}

		this->GetModelPointClosestToActor( _actor_pose, model_ptr->BoundingBox(), model_ptr->GetName() );

		// what if velocity is actually non-zero but Gazebo sees 0?
		f_alpha_beta = this->GetInteractionComponent(	_actor_pose,
														_actor_velocity,
														model_ptr->WorldPose(),
														model_vel,
														is_an_actor);
		f_interaction += f_alpha_beta;
		if ( print_info ) {
			std::cout << " model's name: " << model_ptr->GetName() << "  pose: " << model_ptr->WorldPose() << "  lin vel: " << model_vel << "  force: " << f_alpha_beta << std::endl;
		}

	}

	if ( print_info ) {
		std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
	}
#endif

//	return (desired_force_factor * f_alpha + interaction_force_factor * f_alpha_beta);

	// truncate the force value to max to prevent strange speedup of an actor
	ignition::math::Vector3d f_total = desired_force_factor * f_alpha + interaction_force_factor * f_interaction;
	f_total.Z(0.0);

	if ( print_info ) {
		std::cout << "!! SocialForce: " << f_total << "\tinternal: " << desired_force_factor * f_alpha << "\tinteraction: " << interaction_force_factor * f_interaction;
	}

	double force_length = f_total.Length();
	if ( force_length > force_max ) {

		f_total = f_total.Normalize() * force_max;

		if ( print_info ) {
			std::cout << "\tTRUNCATED";
		}

	} else if ( force_length < force_min ) {

		f_total = f_total.Normalize() * force_min;

		if ( print_info ) {
			std::cout << "\tEXTENDED";
		}

	}

	if ( print_info ) {
		std::cout << "\t\tfinalValue: " << f_total << "\tlength: " << f_total.Length() << std::endl;
	}
	return (f_total);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetModelPointClosestToActor(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Box &_bb,
		const std::string &_model_name) // debug only
{

#ifdef DEBUG_BOUNDING_BOX
	if ( print_info ) {
		std::cout << "GetModelPointClosestToActor()" << "\tname: " << _model_name << "\tcenter: " << _bb.Center() << "\tmax: " << _bb.Max() << "\tmin: " << _bb.Min() << std::endl;
	}
#endif

	return ignition::math::Vector3d(0.0, 0.0, 0.0);

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

#ifdef DEBUG_INTERNAL_ACC
	if ( print_info ) {
		std::cout << std::endl;
		std::cout << "---------------------------------------------------------------------------------\n";
		std::cout << "GetInternalAcceleration(): ";
		std::cout << "\tactor_pos: " << _actor_pose.Pos();
		std::cout << "\ttarget: " << _actor_target << "   to_goal_direction: " << to_goal_direction;
		std::cout << "\n\t\t\t\tideal_vel_vector: " << ideal_vel_vector;
		std::cout << "\tf_alpha: " << f_alpha;
		std::cout << std::endl;
		std::cout << std::endl;
	}
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


	/* Another inconsistency between 2011 and 2014 papers connected to Rudloff's SFM version is n_alpha issue.
	 * In 2011 original paper there is said that n_alpha is "pointing in the opposite direction to the walking
	 * direction (deceleration force)".
	 * On the other hand in 2014 paper (that Rudloff is co-author of) they say: "n α is the direction of movement
	 * of pedestrian α". */

	// all calculations here are based on world coordinate system data
	ignition::math::Angle yaw_norm(this->GetYawFromPose(_actor_pose));
	yaw_norm.Normalize();

#ifdef N_ALPHA_V2011
	yaw_norm -= yaw_norm.Pi; // opposite pointing
#endif

	yaw_norm.Normalize();

	ignition::math::Vector3d n_alpha;
	n_alpha.X(+sin(yaw_norm.Radian()));
	n_alpha.Y(-cos(yaw_norm.Radian())); // TODO: why there is a need to negate the value? is it start-pose-dependent?
	n_alpha.Z(0.0); // in-plane movement only at the moment
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

ignition::math::Vector3d SocialForceModel::GetInteractionComponent(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_vel,
		// const ignition::math::Pose3d &_actor_target,
		// const ignition::math::Vector3d &_n_alpha,
		// const SFMObjectType &_object_type,
		const ignition::math::Pose3d &_object_pose,
		const ignition::math::Vector3d &_object_vel,
		//const ignition::math::Box &_object_bb)
		const bool &_is_actor
)
{

	ignition::math::Vector3d d_alpha_beta = _object_pose.Pos() - _actor_pose.Pos();
	// TODO: adjust Z according to stance
	d_alpha_beta.Z(0.0); // it is assumed that all objects are in the actor's plane

	ignition::math::Vector3d n_alpha = this->GetNormalToAlphaDirection(_actor_pose);

	// if the object is not considered as a point - then perform some calculations
	ignition::math::Vector3d f_alpha_beta = this->GetObjectsInteractionForce(_actor_pose,
													_actor_vel, _object_pose, _object_vel,
													n_alpha, d_alpha_beta, _is_actor);
	return f_alpha_beta;


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
		const ignition::math::Vector3d &_sf_vel) {

	// -----------------------------------------
	// STEP 1
	/* NOTE: coordinate systems of the actor and the world are oriented differently (actor's one rotated 90 deg CCW)
	 * World's yaw is expressed as ((IGN_PI / 2) + local_yaw_angle) where world's yaw is the angle between the X-axis
	 * and a velocity vector.
	 * yaw_target converted to the actor coordinate system to tell the difference */
	ignition::math::Angle yaw_target(std::atan2(_sf_vel.Y(), _sf_vel.X()) + (IGN_PI/2));
	yaw_target.Normalize();

	//double yaw_start = _actor_pose.Rot().Yaw();
	double yaw_start = GetYawFromPose(_actor_pose);
	ignition::math::Angle yaw_diff( yaw_start - yaw_target.Radian() );	// correct

	/* With the below version there is such a debug info printed:
	 * yaw_start: -2.761	yaw_target: 0.380506	angle_change: -0.001	yaw_new: -2.762
	 * and NO FURTHER CHANGE - actor ends up with back facing the target
	ignition::math::Angle yaw_diff( yaw_target.Radian() - yaw_start );
	*/
	yaw_diff.Normalize();

	// -----------------------------------------
	// STEP 2
	// smooth the rotation if too big
	// avoid big yaw changes, force smooth rotations; truncate to max
	static const double YAW_INCREMENT = 0.001;
	short int sign = -1;

	/* check the sign of the yaw_diff - the rotational movement should be performed
	 * in the OPPOSITE direction to yaw_diff angle */
	if ( yaw_diff.Radian() < 0.0f ) {
		sign = +1;
	}

	// save the change to tell if actor is already aligned or not
	double angle_change = 0.0;

	// consider the difference (increment or decrement)
	if ( std::fabs(yaw_diff.Radian()) < YAW_INCREMENT ) {
		angle_change = static_cast<double>(sign) * yaw_diff.Radian();
	} else {
		angle_change = static_cast<double>(sign) * YAW_INCREMENT;
	}

	// -----------------------------------------
	// STEP 3
	ignition::math::Angle yaw_new;
	// yaw_new.Radian(_actor_pose.Rot().Euler().Z() + angle_change);
	yaw_new.Radian(yaw_start + angle_change);
	yaw_new.Normalize();

	// END

	// ----------------------------------------------
	// debug
	if ( print_info ) {
		std::cout << "\t\tyaw_start: " << _actor_pose.Rot().Euler().Z() << "\tyaw_target: " << yaw_target.Radian() << "\tangle_change: " << angle_change << "\tyaw_new: " << yaw_new.Radian() << std::endl;
	}
	// ----------------------------------------------

	return yaw_new;

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
	ignition::math::Angle yaw_new = this->GetYawMovementDirection(_actor_pose, result_vel);

	/* calculate velocity components according to the yaw_new (that was likely truncated to prevent jumps
	 * in rotational movement - this provides smoothed rotation)
	 * only on-plane motions are supported, thus X and Y calculations */
	result_vel.X( +sin(yaw_new.Radian()) * result_vel.SquaredLength() );
	result_vel.Y( -cos(yaw_new.Radian()) * result_vel.SquaredLength() );

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
				 yaw_new.Radian());


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
	 * everyone moving in slightly other way */

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
		std::cout << "  spd_rel: " << rel_vel.SquaredLength();
		std::cout << std::endl;
	}

	return rel_vel.SquaredLength();

#else

	return ( (_object_velocity - _actor_velocity).SquaredLength() );

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

ignition::math::Vector3d SocialForceModel::GetObjectsInteractionForce(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_velocity,
		const ignition::math::Pose3d &_object_pose,
		const ignition::math::Vector3d &_object_velocity,
		const ignition::math::Vector3d &_n_alpha, 		// actor's normal (based on velocity vector)
		const ignition::math::Vector3d &_d_alpha_beta, 	// vector between objects positions
		const bool &_is_actor
)
{

#ifdef DEBUG_INTERACTION_FORCE
	if ( print_info ) {
		std::cout << "GetObjectsInteractionForce(): ";
	}
#endif

	// TODO: only 6 closest actors taken into consideration?
	ignition::math::Vector3d f_alpha_beta(0.0, 0.0, 0.0);

	/* Force is zeroed in 2 cases:
	 * 	- distance between actors is bigger than 5 meters
	 * 	- the other actor is more than 0.5 m behind */
	double d_alpha_beta_length = _d_alpha_beta.Length();
	if ( d_alpha_beta_length > 10.0 ) {

		// TODO: if no objects nearby the thershold should be increased
		#ifdef DEBUG_INTERACTION_FORCE
		if ( print_info ) {
			std::cout << "\t OBJECT TOO FAR AWAY, ZEROING FORCE! \t d_alpha_beta_length: " << d_alpha_beta_length;
			std::cout << std::endl;
		}
		#endif

		return f_alpha_beta;
	}

	/* yaw of an actor is always updated in new pose calculation procedure, so setting the yaw
	 * based on world info should work - actor is always oriented in his movement direction
	 * (if linear speed is non-zero) */
	ignition::math::Angle actor_yaw(GetYawFromPose(_actor_pose));
	actor_yaw.Normalize();

	/* this is a simplified version - object's yaw could be taken from world's info indeed,
	 * but the object's coordinate system orientation is not known, so velocity calculation
	 * still may need to be performed (if needed depending on θ αβ calculation method);
	 * when `_is_actor` flag is set no further velocity calculation needed! */
	ignition::math::Angle object_yaw(GetYawFromPose(_object_pose));
	object_yaw.Normalize();

	uint8_t beta_rel_location = this->GetBetaRelativeLocation(actor_yaw, _d_alpha_beta);

	if ( beta_rel_location == SFM_BEHIND && d_alpha_beta_length > 0.5 ) {

		#ifdef DEBUG_INTERACTION_FORCE
		if ( print_info ) {
			std::cout << "\t OBJECT FAR BEHIND, ZEROING FORCE!";
			std::cout << std::endl;
		}
		#endif

		return f_alpha_beta;
	}

	double v_rel = GetRelativeSpeed(_actor_velocity, _object_velocity);

	if ( v_rel < 1e-06 ) {

		#ifdef DEBUG_INTERACTION_FORCE
		if ( print_info ) {
			std::cout << "  v_rel = 0, ZEROING FORCE!";
			std::cout << std::endl;
		}
		#endif

		return f_alpha_beta;

	}

	/* There is an inconsistency in papers connected with the Rudloff's version of Social Force model -
	 * in Rudloff et al. 2011 - https://www.researchgate.net/publication/236149039_Can_Walking_Behavior_Be_Predicted_Analysis_of_Calibration_and_Fit_of_Pedestrian_Models
	 * there is a statement that theta_alpha_beta is an "angle between velocity of pedestrian α and the displacement of pedestrian β"
	 * whereas in Seer et al. 2014 - https://www.sciencedirect.com/science/article/pii/S2352146514001161
	 * they say that in this model "φ αβ is the angle between n α and d αβ" (they call it phi instead of theta) */

#if		defined(THETA_ALPHA_BETA_V2011)
	double theta_alpha_beta = this->GetAngleBetweenObjectsVelocities(_actor_velocity, actor_yaw, _object_velocity, object_yaw, _is_actor);
#elif 	defined(THETA_ALPHA_BETA_V2014)
	double theta_alpha_beta = this->GetAngleAlphaBeta(_n_alpha, _d_alpha_beta);
#else
	/* NOTE: below method of calculating the angle is only correct when both objects are:
	 * 		o dynamic,
	 * 		o currently moving,
	 * 		o already aligned with the to-target-direction,
	 * 		o there are no obstacles in the environment that will make the object not move along a straight line. */
	double theta_alpha_beta = this->GetAngleBetweenObjectsVelocities(_actor_pose, &actor_yaw, _object_pose, &object_yaw);
#endif

	ignition::math::Vector3d p_alpha = GetPerpendicularToNormal(_n_alpha, beta_rel_location); 	// actor's perpendicular (based on velocity vector)
	double exp_normal = ( (-Bn * theta_alpha_beta * theta_alpha_beta) / v_rel ) - Cn * d_alpha_beta_length;
	double exp_perpendicular = ( (-Bp * std::fabs(theta_alpha_beta) ) / v_rel ) - Cp * d_alpha_beta_length;
	f_alpha_beta = _n_alpha * An * exp(exp_normal) + p_alpha * Ap * exp(exp_perpendicular);

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

	return f_alpha_beta;

}

// ------------------------------------------------------------------- //

// one of 3 possibilities of calculating theta_alpha_beta will be chosen

#if !defined(THETA_ALPHA_BETA_V2011) && !defined(THETA_ALPHA_BETA_V2014)

// dynamic objects interaction
double SocialForceModel::GetAngleBetweenObjectsVelocities(
		const ignition::math::Pose3d &_actor_pose,
		ignition::math::Angle *_actor_yaw,
		const ignition::math::Pose3d &_object_pose,
		ignition::math::Angle *_object_yaw)
{

	// only on-plane movement considered

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "GetAngleBetweenObjectsVelocities(): ";
	}
#endif

#ifdef THETA_ALPHA_BETA_CONSIDER_ZERO_VELOCITY

	// check if objects are moving
	float actor_speed = _actor_vel.X()*_actor_vel.X() + _actor_vel.Y()*_actor_vel.Y();
	if ( actor_speed < 1e-3 ) {
#ifdef DEBUG_GEOMETRY_1
		if ( print_info ) {
			std::cout << " ACTOR'S SPEED is close to 0! " << std::endl;
		}
#endif
		return 0.0;
	}

	float object_speed = _object_vel.X()*_object_vel.X() + _object_vel.Y()*_object_vel.Y();
	if ( object_speed < 1e-3 ) {
#ifdef DEBUG_GEOMETRY_1
		if ( print_info ) {
			std::cout << " OBJECT'S SPEED is close to 0! " << std::endl;
		}
#endif
		return 0.0;
	}

	//actor_speed  = sqrt(actor_speed);
	//object_speed = sqrt(object_speed);

	// velocities are not 0 so calculate the angle between those 2 vectors

#endif

//	ignition::math::Vector3d rpy_actor = _actor_pose.Rot().Euler();
//	ignition::math::Vector3d rpy_object = _object_pose.Rot().Euler();
//
//	_actor_yaw->Radian(rpy_actor.Z());
//	_actor_yaw->Normalize();
//
//	_object_yaw->Radian(rpy_object.Z());
//	_object_yaw->Normalize();



//	// this is already done before the function invocation!		!
//	_actor_yaw->Radian(this->GetYawFromPose(_actor_pose));		!
//	_actor_yaw->Normalize();									!
//																!
//	_object_yaw->Radian(this->GetYawFromPose(_object_pose));	!
//	_object_yaw->Normalize();									!


	// ignition::math::Angle yaw_diff = *_object_yaw - *_actor_yaw;					// TODO: below version is explicit, but is it correct? - DEBUG

//	ignition::math::Angle yaw_diff(_object_yaw->Radian() - _actor_yaw->Radian());
	ignition::math::Angle yaw_diff(_actor_yaw->Radian() - _object_yaw->Radian());
	yaw_diff.Normalize();

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "\t yaw_actor: " << _actor_yaw->Radian() << "  yaw_object: " << _object_yaw->Radian() << "  yaw_diff: " << yaw_diff.Radian() << std::endl;
	}
#endif

	return yaw_diff.Radian();

}

#endif

// ------------------------------------------------------------------- //

#if defined(THETA_ALPHA_BETA_V2011)

// 2011 - "θ αβ - angle between velocity of pedestrian α and the displacement of pedestrian β"
double SocialForceModel::GetAngleBetweenObjectsVelocities(
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
	 * On the other hand the object's orientation is not known and its velocity
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
		yaw_diff.Radian((_actor_yaw.Radian() - _object_yaw.Radian()));
		yaw_diff.Normalize();

	} else {

		/* so first let's check if the considered object is static or dynamic;
		 * if it is static then the only thing that could be done is calculating
		 * the yaw_diff as in `_is_actor` case */
		if ( _object_vel.Length() < 1e-06 ) {

			#ifdef DEBUG_GEOMETRY_1
			if ( print_info ) {
				std::cout << "\t++STATIC object++";
			}
			#endif

			yaw_diff.Radian((_actor_yaw.Radian() - _object_yaw.Radian()));
			yaw_diff.Normalize();

		} else {

			#ifdef DEBUG_GEOMETRY_1
//			if ( print_info ) {
				std::cout << "\t++!!! DYNAMIC object !!!++";
//			}
			#endif

			// TODO: debug this, NOT TESTED!
			// velocities are expressed in world's coordinate system

			// transform object's yaw to actor's coordinate system by adding 90 deg
			ignition::math::Angle yaw_temp( std::atan2( _object_vel.Y(), _object_vel.X() ) + (IGN_PI/2) );
			yaw_temp.Normalize();

			yaw_diff.Radian( _actor_yaw.Radian() - yaw_temp.Radian() );
			yaw_diff.Normalize();

		}

	}


#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "\t yaw_actor: " << _actor_yaw.Radian() << "  yaw_object: " << _object_yaw.Radian() << "  yaw_diff: " << yaw_diff.Radian() << std::endl;
	}
#endif

	return (yaw_diff.Radian());

}

#endif

// ------------------------------------------------------------------- //

#if defined(THETA_ALPHA_BETA_V2014)

// 2014 - "φ αβ is the angle between n α and d αβ"
double SocialForceModel::GetAngleAlphaBeta(
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

#endif

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetPerpendicularToNormal(

		const ignition::math::Vector3d &_n_alpha,
		const uint8_t &_beta_rel_location)

{

	/* Depending on which side beta is on relative to n_alpha, the p_alpha (perpendicular vector)
	 * will point to direction opposite to the side where beta is */

	/*
	ignition::math::Vector3d p_alpha;
	if ( _beta_rel_location == SFM_RIGHT_SIDE ) {
		p_alpha = _n_alpha.Perpendicular();
		// return (_n_alpha.Perpendicular());
	} else if ( _beta_rel_location == SFM_LEFT_SIDE ) {

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

	if ( _beta_rel_location == SFM_LEFT_SIDE ) {

		#if 	defined(N_ALPHA_V2011)
		to_cross.Set(0.0, 0.0, -1.0);
		#elif 	defined(N_ALPHA_V2014)
		to_cross.Set(0.0, 0.0,  1.0);
		#endif

	} else if ( _beta_rel_location == SFM_RIGHT_SIDE ) {

		#if 	defined(N_ALPHA_V2011)
		to_cross.Set(0.0, 0.0,  1.0);
		#elif 	defined(N_ALPHA_V2014)
		to_cross.Set(0.0, 0.0, -1.0);
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

uint8_t SocialForceModel::GetBetaRelativeLocation(
		const ignition::math::Angle &_actor_yaw,
		const ignition::math::Vector3d &_d_alpha_beta)
{

	uint8_t rel_loc = 0;
	ignition::math::Angle angle_relative; 		// relative to actor's (alpha) direction
	ignition::math::Angle angle_d_alpha_beta;	// stores yaw of d_alpha_beta

	ignition::math::Vector3d d_alpha_beta_norm = _d_alpha_beta;
	d_alpha_beta_norm.Normalize();

	// when normalized vector used with atan2 then division by euclidean distance not needed
	angle_d_alpha_beta.Radian( std::atan2(d_alpha_beta_norm.X(), d_alpha_beta_norm.Y()) );
	angle_d_alpha_beta.Normalize();

	angle_relative = _actor_yaw + angle_d_alpha_beta;
	angle_relative.Normalize();

#ifdef DEBUG_GEOMETRY_2
	if ( print_info ) {
		std::cout << "\n GetBetaRelativeLocation() " << "ACTOR yaw: " << _actor_yaw.Radian() << "  ANGLE d_alpha_beta: " << angle_d_alpha_beta.Radian() << "  ANGLE sum: " << angle_relative.Radian();
	}
	std::string txt_dbg;
#endif

	// consider FOV
	if ( IsOutOfFOV(angle_relative.Radian() ) ) {

		rel_loc = SFM_BEHIND;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "BEHIND";
#endif

	} else if ( angle_relative.Radian() <= 0.0 ) {

		rel_loc = SFM_RIGHT_SIDE;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "RIGHT";
#endif

	} else if ( angle_relative.Radian() > 0.0 ) {

		rel_loc = SFM_LEFT_SIDE;
#ifdef DEBUG_GEOMETRY_2
		txt_dbg = "LEFT";
#endif

	}

#ifdef DEBUG_GEOMETRY_2
	if ( print_info ) {
		std::cout << "  " << txt_dbg << std::endl;
	}
#endif

	return rel_loc;

}

bool SocialForceModel::IsOutOfFOV(const double &_angle_relative) {

	if ( std::fabs(_angle_relative) <= fov ) {
		return true;
	}

	return false;

}

// ------------------------------------------------------------------- //

SocialForceModel::~SocialForceModel() { }

}
