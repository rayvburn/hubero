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
// #define DEBUG_GEOMETRY_1 // angle correctes etc.
#define DEBUG_GEOMETRY_2 // relative location
// #define DEBUG_INTERNAL_ACC
#define DEBUG_INTERACTION_FORCE
#define DEBUG_REL_SPEED
// #define DEBUG_NEW_POSE
#define DEBUG_ACTOR_FACING_TARGET

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
	desired_force_factor(1300.0), interaction_force_factor(200.0) {

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
		}
	} else {
		print_info = false;
	}

	// allocate all variables needed in loop
	ignition::math::Vector3d f_alpha = this->GetInternalAcceleration(_actor_pose,
																	 _actor_velocity,
																	 _actor_target);
	ignition::math::Vector3d f_alpha_beta(0.0, 0.0, 0.0);
	ignition::math::Vector3d f_ab_single(0.0, 0.0, 0.0);

	/* model_vel contains model's velocity (world's object or actor) - for the actor this is set differently
	 * it was impossible to set actor's linear velocity by setting it by the model's class method */
	ignition::math::Vector3d model_vel;

	gazebo::physics::ModelPtr model_ptr;
	/*
	for ( unsigned int i = 0; i < _world_ptr->ModelCount(); i++ ) {

		model_ptr = _world_ptr->ModelByIndex(i);

		if ( model_ptr->GetName() == _actor_name ) {
			// do not calculate social force for itself
			continue;
		}

		if ( print_info ) {
			std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
		}

		if ( this->IsActor(model_ptr->GetName()) ) {
			model_vel = GetActorVelocity(model_ptr, _map_actor_name_id, _actors_velocities);
		} else {
			model_vel = model_ptr->WorldLinearVel();
		}

		// what if velocity is actually non-zero but Gazebo sees 0?
		f_ab_single += this->GetInteractionComponent(	_actor_pose,
														_actor_velocity,
														model_ptr->WorldPose(),
														model_vel);
		f_alpha_beta += f_ab_single;
		if ( print_info ) {
			std::cout << " model's name: " << model_ptr->GetName() << "  pose: " << model_ptr->WorldPose() << "  lin vel: " << model_vel << "  force: " << f_ab_single << std::endl;
		}

	}

	if ( print_info ) {
		std::cout << ":::::::::::::::::::::::::::::::::::::::::::::::::::" << std::endl;
	}
	*/
	return (desired_force_factor * f_alpha + interaction_force_factor * f_alpha_beta);


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

	ignition::math::Angle yaw_norm(_actor_pose.Rot().Euler().Z());
	// yaw_norm.Radian(rpy.Z());
	yaw_norm.Normalize();
	yaw_norm -= yaw_norm.Pi; // opposite pointing
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
		const ignition::math::Vector3d &_object_vel
		//const ignition::math::Box &_object_bb)
)
{

	ignition::math::Vector3d d_alpha_beta = _object_pose.Pos() - _actor_pose.Pos();
	// TODO: adjust Z according to stance
	d_alpha_beta.Z(0.0); // it is assumed that all objects are in the actor's plane

	ignition::math::Vector3d n_alpha = this->GetNormalToAlphaDirection(_actor_pose);

	// if the object is not considered as a point - then perform some calculations
	ignition::math::Vector3d f_alpha_beta = this->GetObjectsInteractionForce(_actor_pose,
													_actor_vel, _object_pose, _object_vel,
													n_alpha, d_alpha_beta);
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

ignition::math::Pose3d SocialForceModel::GetNewPose(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Pose3d &_actor_last_pose,
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Vector3d &_actor_target,
			const ignition::math::Vector3d &_social_force,
			const double &_dt,
			const uint8_t &_stance)
{

	bool recalculate_vel = false;
	ignition::math::Pose3d new_pose;

	// a = F/m
	ignition::math::Vector3d result_vel = (_social_force / this->mass_person) * _dt;

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
		std::cout << "GetNewPose(): ";
		std::cout << "\tresult_vel: " << result_vel;
	}
#endif

	/* If calculated speed value is bigger than max speed -> perform normalization
	// leave velocity direction as is, shorten the vector to max possible */
	if ( result_vel.Length() > speed_max ) {
		result_vel = result_vel.Normalize() * speed_max;

		#ifdef DEBUG_NEW_POSE
		if ( print_info ) {
			std::cout << "\t vel TRUNCATED!: " << result_vel;
		}
		#endif

	}

	// temporary vector caused by social-force-based displacements
	ignition::math::Vector3d new_position(_actor_pose.Pos().X() + result_vel.X() * _dt,
										  _actor_pose.Pos().Y() + result_vel.Y() * _dt,
										  _actor_pose.Pos().Z());

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
		std::cout << "\nPOSITION1 \torig: " << _actor_pose.Pos() << "\tdelta_x: " << result_vel.X() * _dt << "\tdelta_y: " << result_vel.Y() * _dt << "\tnew_position: " << new_position << std::endl;
	}
#endif

/*
	// to smooth the rotation
	ignition::math::Angle yaw_vel_based(atan2( result_vel.X(), result_vel.Y() ));
	// yaw_delta.Radian(yaw_delta.Radian() + (IGN_PI / 2));
	yaw_vel_based.Normalize();

	// yaw is expressed as ((IGN_PI / 2) + yaw_angle) where yaw_angle is between x axis line and velocity vector
	if ( std::abs(_actor_pose.Rot().Euler().Z() - (IGN_PI / 2) - yaw_vel_based.Radian()) > IGN_DTOR(10) ) {
		if ( print_info ) {
			std::cout << "\t yaw TRUNCATED!: " << yaw_vel_based.Radian();
		}
		yaw_vel_based.Radian(yaw_vel_based.Radian() * 0.001);
		recalculate_vel = true;
	}

	ignition::math::Angle yaw_new(yaw_vel_based.Radian() + (IGN_PI/2));
	// assuming standing pose when sf is calculated thus roll angle is set to half-pi (STANDING)
	ignition::math::Vector3d new_orientation((IGN_PI/2), 0, yaw_new.Radian());
*/

// ---------------------------------------------------
	/// Total yaw is expressed as ((IGN_PI / 2) + yaw_angle) where yaw is the angle between x axis line and velocity vector
	// #ifdef NEW_YAW_BASE

	// NOTE: coordinate systems of the actor and the world are orientated differently (actor's one rotated 90 deg CCW)
	// V2a
	/*
//	ignition::math::Angle yaw_delta(std::atan2(new_position.Y(), new_position.X()) + (IGN_PI / 2) - _actor_pose.Rot().Euler().Z());
	ignition::math::Angle yaw_delta(std::atan2(result_vel.Y(), result_vel.X()) + (IGN_PI / 2) - _actor_pose.Rot().Euler().Z());

	yaw_delta.Normalize();
	*/

	// V2b
	ignition::math::Angle yaw_target(std::atan2(result_vel.Y(), result_vel.X()) + (IGN_PI/2));
	yaw_target.Normalize();

	double yaw_start = _actor_pose.Rot().Yaw();
	ignition::math::Angle yaw_diff( yaw_start - yaw_target.Radian() );
	yaw_diff.Normalize();


	// ----------------------------------------------
	// V1
	/// Smooth rotation
//	ignition::math::Angle yaw_new;
//	if (std::fabs(yaw_delta.Radian()) > IGN_DTOR(10)) {
//
//      yaw_new.Radian(_actor_pose.Rot().Euler().Z() + yaw_delta.Radian() * 0.001);
//      if ( print_info ) {
//          std::cout << "\n\n\n\tSMOOTHING ROTATION\tyaw_initial: " << _actor_pose.Rot().Euler().Z() << /*"\tyaw_2: " << yaw_2.Radian() << */"\tyaw_delta: " << yaw_delta.Radian() << "\tyaw_new: " << yaw_new.Radian() << "\n\n\n" << std::endl;
//      }
//      recalculate_vel = true;
//
//	} else {
//
//	  yaw_new.Radian(_actor_pose.Rot().Euler().Z() + yaw_delta.Radian());
//	  if ( print_info ) {
//		  std::cout << "\n\tNORMAL ROTATION\tyaw_initial: " << _actor_pose.Rot().Euler().Z() << /*"\tyaw_2: " << yaw_2.Radian() << */"\tyaw_delta: " << yaw_delta.Radian() << "\tyaw_new: " << yaw_new.Radian() << '\n' << std::endl;
//	  }
//
//	}

	// ----------------------------------------------
	// V2a
	/*
	// check the sign of the diff - the movement should be performed in the OPPOSITE direction to yaw_diff angle
	static const double YAW_INCREMENT = 0.001; // 0.001; - OK - smooth
//	short int sign = -1;
//	if ( yaw_delta.Radian() < 0.0f ) {
//		sign = +1;
//	}

	// inverted sign?
	short int sign = +1;
	if ( yaw_delta.Radian() < 0.0f ) {
		sign = -1;
	}

	// save the change to tell if actor is already aligned or not
	double angle_change = 0.0;

	// consider the difference (increment or decrement)
	if ( std::fabs(yaw_delta.Radian()) < YAW_INCREMENT ) {
		angle_change = static_cast<double>(sign) * yaw_delta.Radian();
	} else {
		angle_change = static_cast<double>(sign) * YAW_INCREMENT;
	}
	*/

	// V2b
	// smooth the rotation if too big
	static const double YAW_INCREMENT = 0.001;
	short int sign = -1;
//	ignition::math::Vector3d rpy = this->pose_actor.Rot().Euler();

	// check the sign of the diff - the movement should be performed in the OPPOSITE direction to yaw_diff angle
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
	// ---------------------------
	ignition::math::Angle yaw_new;
	yaw_new.Radian(_actor_pose.Rot().Euler().Z() + angle_change);
	yaw_new.Normalize();

	// debug
	if ( print_info ) {
		// yaw target expressed as a yaw of new coordinates
//		std::cout << "\t\tyaw_start: " << _actor_pose.Rot().Euler().Z() << "\tyaw_target: " << std::atan2(new_position.Y(), new_position.X()) << "\tangle_change: " << angle_change << "\tyaw_delta: " << yaw_delta.Radian() << "\tyaw_new: " << yaw_new.Radian() << std::endl;

// V2a	std::cout << "\t\tyaw_start: " << _actor_pose.Rot().Euler().Z() << "\tyaw_target: " << std::atan2(result_vel.Y(), result_vel.X()) << "\tangle_change: " << angle_change << "\tyaw_delta: " << yaw_delta.Radian() << "\tyaw_new: " << yaw_new.Radian() << std::endl;
		std::cout << "\t\tyaw_start: " << _actor_pose.Rot().Euler().Z() << "\tyaw_target: " << yaw_target.Radian() << "\tangle_change: " << angle_change << "\tyaw_new: " << yaw_new.Radian() << std::endl;
	}

	// ----------------------------------------------




	/* TODO: debug
	// if the actor's yaw angle is such that he faces opposite direction to his target - make him rotate in place first
	if ( IsActorFacingTheTarget(yaw_new, _actor_target) ) {

		// ok - don't rotate in place
		#ifdef DEBUG_ACTOR_FACING_TARGET
		if ( print_info ) {
			std::cout << "\tYES!" << std::endl;
		}
		#endif

	} else {

		// rotate in place condition
		recalculate_vel = false;

		// TODO: circular motion - swing around
		result_vel.X(0.0);
		result_vel.Y(0.0);

		// check if delta is non-zero (to artificially force actor's rotation)
		short int sign = 0;

		if ( std::abs(yaw_delta.Radian()) <= 1e-03 ) {

			// the sign of the trend must be determined (or just use yaw_delta's sign (?)
			if ( GetYawFromPose(_actor_pose) - GetYawFromPose(_actor_last_pose) < 0 ) {
				sign = -1;
			} else {
				sign = +1;
			}
			yaw_new.Radian(sign * 1.001f * GetYawFromPose(_actor_last_pose)); 	// speed up the rotation a bit

		} else {

			yaw_new.Radian(0.001 * yaw_delta.Radian() + yaw_new.Radian()); 		// speed up the rotation a bit


		}

		// yaw_new.Radian(sign * 1.01f * GetYawFromPose(_actor_last_pose)); // speed up the rotation a bit
		yaw_new.Normalize();

		#ifdef DEBUG_ACTOR_FACING_TARGET
		if ( print_info ) {
			std::cout << "\tNO!" << "\tyaw_calc: " << yaw_new << "\tsign: " << sign << std::endl;
		}
		#endif

	}
	*/

	/// Recalculate velocity vectors if yaw was truncated
	if ( recalculate_vel ) {
		// TODO: debug sign of a sine/cosine
		result_vel.X(+sin(yaw_new.Radian())*result_vel.SquaredLength());
		result_vel.Y(+cos(yaw_new.Radian())*result_vel.SquaredLength());
		/*
		// 2nd method of yaw_delta calculation
		result_vel.X(+cos(yaw_new.Radian())*result_vel.SquaredLength());
		result_vel.Y(+sin(yaw_new.Radian())*result_vel.SquaredLength());
		*/
		if ( print_info ) {
		    std::cout << "\n\tSMOOTHING ROTATION - RECALCULATED VEL\tx: " << result_vel.X() << "\ty: " << result_vel.Y() << '\n' << std::endl;
		}
	}

	// Set new pose values
	new_pose.Set(_actor_pose.Pos().X() + result_vel.X() * _dt,
				 _actor_pose.Pos().Y() + result_vel.Y() * _dt,
				 1.2138,
				 (IGN_PI/2),
				 0,
				 //yaw_new.Radian());
				 _actor_pose.Rot().Yaw());	// WIP - testing AlignToTarget!

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
//		std::cout << "\nPOSITION2 \t orig: " << _actor_pose.Pos() << " \t delta_x: " << result_vel.X() * _dt << "    delta_y: " << result_vel.Y() * _dt << std::endl;
		std::cout << "\nPOSITION2 \torig: " << _actor_pose.Pos() << "\tdelta_x: " << result_vel.X() * _dt << "\tdelta_y: " << result_vel.Y() * _dt << "\tnew_position: " << new_pose.Pos(); // << std::endl;
	}
#endif

	//new_pose.Pos().X(_actor_pose.Pos().X() + result_vel.X() * _dt);
	//new_pose.Pos().Y(_actor_pose.Pos().Y() + result_vel.Y() * _dt);
	//new_pose.Pos().Z(_actor_pose.Pos().Z()); // + result_vel.X() * _dt);

//	new_pose.Set(new_position, ignition::math::Quaterniond(new_orientation.X(),
//														   new_orientation.Y(),
//														   new_orientation.Z()));

//	new_pose.Set(new_position, _actor_pose.Rot());

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
		std::cout << std::endl;
		std::cout << "---------------------------------------------------------------------------------";
		std::cout << std::endl;
	}
#endif

	return new_pose;

	/* Now consider the yaw angle of an actor - it is crucial to make him face his
	 * current movement direction */

	// TODO: check yaw angle calculation with default Gazebo script?
	ignition::math::Vector3d actor_rpy_initial = _actor_pose.Rot().Euler();
//	ignition::math::Angle yaw_new = atan2( result_vel.X(), result_vel.Y() );
//	ignition::math::Angle yaw_delta = yaw_delta.HalfPi - yaw_new.Radian() - actor_rpy_initial.Z(); // HalfPi - theta(t=i+1) - theta(t=i)
//	yaw_delta.Normalize();

	ignition::math::Angle yaw_delta2 = atan2(result_vel.Y(), result_vel.X()) + 0.5*M_PI - actor_rpy_initial.Z();
	yaw_delta2.Normalize();

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
		std::cout << "\t rpy_initial: " << actor_rpy_initial;
	//	std::cout << "  yaw_new: " << yaw_new;
		std::cout << "\t yaw_delta2: " << yaw_delta2;
	}
#endif

	// avoid big yaw changes, force smooth rotations; truncate to max

//
//	if ( abs( yaw_delta.Radian() ) > yaw_max_delta ) {
//		(yaw_delta < 0) ? (yaw_delta = -yaw_max_delta) : (yaw_delta = +yaw_max_delta);
//
//		#ifdef DEBUG_NEW_POSE
//		std::cout << "  yaw_delta TRUNCATED!: " << yaw_delta;
//		#endif
//
//	}

	ignition::math::Angle yaw_new2;
	yaw_new2.Radian(actor_rpy_initial.Z() + yaw_delta2.Radian() - 0.5*M_PI);

	// update yaw
//	yaw_new += yaw_delta;
	yaw_new2.Normalize();

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
		std::cout << "\t yaw_new2: " << yaw_new2;
	}
#endif

	// calculate x and y velocity components based on new yaw angle (on-plane motions only)
	result_vel.X(+sin(yaw_new2.Radian())*result_vel.SquaredLength());
	result_vel.Y(+cos(yaw_new2.Radian())*result_vel.SquaredLength());

#ifdef DEBUG_NEW_POSE
	if ( print_info ) {
		std::cout << "\n \t vel YAW-CORRECTED: " << result_vel;
	}
#endif

	// calculate new pose - consider current pose, velocity and delta of time
	// set new rotation in the pose vector
	ignition::math::Vector3d new_pos;
	new_pos.X( _actor_pose.Pos().X() + result_vel.X()*_dt );
	new_pos.Y( _actor_pose.Pos().Y() + result_vel.Y()*_dt );
	new_pos.Z( _actor_pose.Pos().Z() + result_vel.Z()*_dt );

	ignition::math::Vector3d new_rot_rpy = _actor_pose.Rot().Euler();
	new_rot_rpy.X(1.5707);
	new_rot_rpy.Z(yaw_new2.Radian());

	//
	// TODO: force pose.Z() according to current 'stance'
	//

	new_pose.Set(new_pos, new_rot_rpy);

	//
	if ( print_info ) {
		print_info = false;
		std::cout << " -------------- PRINT_INFO triggered FALSE ------------------ " << std::endl;
	}
	//

	return new_pose;

}

// ------------------------------------------------------------------- //

// overload?
//ignition::math::Pose3d SocialForceModel::GetNewPose(
//			const ignition::math::Pose3d &_actor_pose,
//			const ignition::math::Vector3d &_actor_vel,
//			const ignition::math::Vector3d &_actor_target,
//			const ignition::math::Vector3d &_social_force,
//			const double &_dt,
//			const uint8_t &_stance)
//{
//
//}

// ------------------------------------------------------------------- //

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

	ignition::math::Vector3d rel_vel = _object_velocity - _actor_velocity;

#ifdef DEBUG_REL_SPEED
	if ( print_info ) {
		std::cout << std::endl;
		std::cout << "GetRelativeSpeed(): ";
		std::cout << "  obj_vel: " << _object_velocity;
		std::cout << "  act_vel: " << _actor_velocity;
		std::cout << "  v_rel:  " << rel_vel;
		std::cout << "  spd_rel: " << rel_vel.SquaredLength();
		std::cout << std::endl;
	}
#endif

	return rel_vel.SquaredLength();;

}

// ============================================================================

inline double SocialForceModel::GetYawFromPose(const ignition::math::Pose3d &_actor_pose) {
	// the actor's offset yaw is considered
	// return (_actor_pose.Rot().Yaw() + (IGN_PI / 2));

	// when offset already considered
	return (_actor_pose.Rot().Yaw());
}

// ============================================================================

ignition::math::Vector3d SocialForceModel::GetObjectsInteractionForce(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_velocity,
		const ignition::math::Pose3d &_object_pose,
		const ignition::math::Vector3d &_object_velocity,
		const ignition::math::Vector3d &_n_alpha, 		// actor's normal (based on velocity vector)
		const ignition::math::Vector3d &_d_alpha_beta 	// vector between objects positions
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

	ignition::math::Angle actor_yaw(GetYawFromPose(_actor_pose));
	ignition::math::Angle object_yaw(0.0);
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

	if ( v_rel < 1e-12 ) {

		#ifdef DEBUG_INTERACTION_FORCE
		if ( print_info ) {
			std::cout << "  v_rel = 0, ZEROING FORCE!";
			std::cout << std::endl;
		}
		#endif

		return f_alpha_beta;

	}

	double theta_alpha_beta = this->GetAngleBetweenObjectsVelocities(_actor_pose, &actor_yaw, _object_pose, &object_yaw);
	ignition::math::Vector3d p_alpha = GetPerpendicularToNormal(_n_alpha, beta_rel_location); 	// actor's perpendicular (based on velocity vector)
	double exp_normal = ((-Bn * theta_alpha_beta * theta_alpha_beta)/v_rel) - Cn * d_alpha_beta_length;
	double exp_perpendicular = ((-Bp * theta_alpha_beta)/v_rel) - Cp * d_alpha_beta_length;
	f_alpha_beta = _n_alpha * An * exp(exp_normal) + p_alpha * Ap * exp(exp_perpendicular);

#ifdef DEBUG_INTERACTION_FORCE
	if ( print_info ) {
		std::cout << "  v_rel: " << v_rel;
		std::cout << "  exp_n: " << exp_normal;
		std::cout << "  exp_p: " << exp_perpendicular;
		std::cout << "  f_alpha_beta: " << f_alpha_beta;
		std::cout << std::endl;
	}
#endif

	return f_alpha_beta;

}

// ------------------------------------------------------------------- //

// dynamic objects interaction
double SocialForceModel::GetAngleBetweenObjectsVelocities(
		const ignition::math::Pose3d &_actor_pose,
		ignition::math::Angle *_actor_yaw,					// alpha's yaw - will be calculated and saved for further work
		const ignition::math::Pose3d &_object_pose,
		ignition::math::Angle *_object_yaw)					// beta's yaw - will be calculated and saved for further work
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

	ignition::math::Vector3d rpy_actor = _actor_pose.Rot().Euler();
	ignition::math::Vector3d rpy_object = _object_pose.Rot().Euler();

	_actor_yaw->Radian(rpy_actor.Z());
	_actor_yaw->Normalize();

	_object_yaw->Radian(rpy_object.Z());
	_object_yaw->Normalize();

	ignition::math::Angle yaw_diff = *_object_yaw - *_actor_yaw;
	yaw_diff.Normalize();

#ifdef DEBUG_GEOMETRY_1
	if ( print_info ) {
		std::cout << "\t yaw_actor: " << _actor_yaw->Radian() << "  yaw_object: " << _object_yaw->Radian() << "  yaw_diff: " << yaw_diff.Radian() << std::endl;
	}
#endif

	return yaw_diff.Radian();

}

// ==================================================================== //


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
		to_cross.Set(0.0, 0.0,  1.0);
	} else if ( _beta_rel_location == SFM_RIGHT_SIDE ) {
		to_cross.Set(0.0, 0.0, -1.0);
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
