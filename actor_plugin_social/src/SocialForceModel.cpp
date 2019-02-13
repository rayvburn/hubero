/*
 * SocialForceModel.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#include <SocialForceModel.h>
#include <cmath>

namespace SocialForceModel {

#define SFM_RIGHT_SIDE 0
#define SFM_LEFT_SIDE  1
#define SFM_BEHIND     2

// #define THETA_ALPHA_BETA_CONSIDER_ZERO_VELOCITY
// #define BETA_REL_LOCATION_BASED_ON_NORMAL

// #define DEBUG_SFM_PARAMETERS
// #define DEBUG_GEOMETRY_1 // angle correctes etc.
#define DEBUG_GEOMETRY_2 // relative location
#define DEBUG_INTERNAL_ACC
#define DEBUG_INTERACTION_FORCE
#define DEBUG_REL_SPEED
#define DEBUG_NEW_POSE

// ------------------------------------------------------------------- //

SocialForceModel::SocialForceModel():

	fov(1.80), speed_max(1.50), yaw_max_delta(20.0 / M_PI * 180.0), mass_person(1),
	desired_force_factor(1.5), interaction_force_factor(1.0) {

	SetParameterValues();

	/* Algorithm PARAMETERS are:
	 * - relaxation time must be given here
	 * - kind of coefficient for attraction artificial potential field (decreases over time)
	 * - FOV
	 * - direction weight
	 * - max speed
	 */

}

// ------------------------------------------------------------------- //

void SocialForceModel::Init(const unsigned short int _mass_person, const float _desired_force_factor, const float _interaction_force_factor) {



}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetSocialForce(
	const gazebo::physics::WorldPtr _world_ptr,
	const std::string _actor_name,
	const ignition::math::Pose3d _actor_pose,
	const ignition::math::Vector3d _actor_velocity,
	const ignition::math::Vector3d _actor_target
) {

	/* IN */


	/* fake variables to determine INPUTS
	gazebo::physics::WorldPtr _world_ptr;
	std::string _actor_name;
	ignition::math::Pose3d _actor_pose;
	ignition::math::Vector3d _actor_velocity;
	ignition::math::Vector3d _actor_target;
	*/

	// allocate all variables needed in loop
	ignition::math::Vector3d f_alpha = this->GetInternalAcceleration(_actor_pose,
																	 _actor_velocity,
																	 _actor_target);
	ignition::math::Vector3d f_alpha_beta(0.0, 0.0, 0.0);
	ignition::math::Vector3d f_ab_single(0.0, 0.0, 0.0);

	// TODO: add f_alpha and f_alpha_beta coefficients

	gazebo::physics::ModelPtr model_ptr;
	for ( unsigned int i = 0; i < _world_ptr->ModelCount(); i++ ) {

		model_ptr = _world_ptr->ModelByIndex(i);

		if ( model_ptr->GetName() == _actor_name ) {
			// do not calculate social force for itself
			continue;
		}

		// what if velocity is actually non-zero but Gazebo sees 0?
		f_ab_single += this->GetInteractionComponent(	_actor_pose,
														_actor_velocity,
														model_ptr->WorldPose(),
														model_ptr->WorldLinearVel());
		f_alpha_beta += f_ab_single;
		std::cout << " model's name: " << model_ptr->GetName() << "  pose: " << model_ptr->WorldPose() << "  lin vel: " << model_ptr->WorldLinearVel() << "  force: " << f_ab_single << std::endl;

	}
	return (desired_force_factor * f_alpha + interaction_force_factor * f_alpha_beta);


}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetInternalAcceleration(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_vel,
		const ignition::math::Vector3d &_actor_target)
{

	// internal acceleration is a first step in SF calculations, n_alpha is useful later on and in this stage

	// TODO: a lot of allocations here
	ignition::math::Vector3d to_goal_vector = (_actor_target - _actor_pose.Pos());
	ignition::math::Vector3d to_goal_direction = to_goal_vector.Normalize();
	ignition::math::Vector3d ideal_vel_vector = speed_desired * to_goal_direction;
	ignition::math::Vector3d f_alpha = mass_person * (1/relaxation_time) * (ideal_vel_vector - _actor_vel);

#ifdef DEBUG_INTERNAL_ACC
	std::cout << "GetInternalAcceleration(): ";
	std::cout << "   target: " << _actor_target << "   to_goal_vector: " << to_goal_vector;
	std::cout << "   ideal_vel_vector: " << ideal_vel_vector;
	std::cout << "   f_alpha: " << f_alpha << std::endl;
#endif

	return f_alpha;

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetNormalToAlphaDirection(const ignition::math::Pose3d &_actor_pose) {

	// when speed is 0 then there is no way of calculating the angle THETA_alpha_beta (0 vector length)
	// better way is calculating normal based on actor's yaw than velocity vector

	// std::cout << "BEFORE -- n_alpha: " << *_n_alpha << "  _actor_vel: " << _actor_vel << std::endl;

	ignition::math::Vector3d rpy = _actor_pose.Rot().Euler();

	ignition::math::Angle yaw_norm;
	yaw_norm.Radian(rpy.Z());
	yaw_norm.Normalize();
	yaw_norm -= yaw_norm.Pi; // opposite pointing
	yaw_norm.Normalize();

	ignition::math::Vector3d n_alpha;
	n_alpha.X(+sin(yaw_norm.Radian()));
	n_alpha.Y(-cos(yaw_norm.Radian())); // TODO: why there is a need to negate the value? is it start-pose-dependent?
	n_alpha.Z(0.0); // in-plane movement only at the moment
	n_alpha.Normalize();

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

ignition::math::Pose3d SocialForceModel::GetNewPose(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Vector3d &_social_force,
			const double &_dt,
			const uint8_t &_stance)
{

	ignition::math::Pose3d new_pose;

	// a = F/m
	ignition::math::Vector3d result_vel = _actor_vel + (_social_force / this->mass_person) * _dt;

#ifdef DEBUG_NEW_POSE
	//std::cout << "GetNewPose(): ";
	std::cout << "result_vel: " << result_vel;
#endif

	/* if calculated speed is bigger than max speed -> perform normalization
	// leave velocity direction as is, shorten the vector to max possible */
	if ( result_vel.Length() > speed_max ) {
		result_vel = result_vel.Normalize() * speed_max;

		#ifdef DEBUG_NEW_POSE
		std::cout << "\t vel TRUNCATED!: " << result_vel;
		#endif

	}

	ignition::math::Vector3d new_position(_actor_pose.Pos().X() + result_vel.X() * _dt,
										  _actor_pose.Pos().Y() + result_vel.Y() * _dt,
										  _actor_pose.Pos().Z());

//	new_pose.Pos().X(_actor_pose.Pos().X() + result_vel.X() * _dt);
//	new_pose.Pos().Y(_actor_pose.Pos().Y() + result_vel.Y() * _dt);
//	new_pose.Pos().Z(_actor_pose.Pos().Z()); // + result_vel.X() * _dt);

	new_pose.Set(new_position, _actor_pose.Rot());
	return new_pose;

	/* Now consider the yaw angle of an actor - it is crucial to make him face his
	 * current movement direction */

	// TODO: check yaw angle calculation with default Gazebo script?
	ignition::math::Vector3d actor_rpy_initial = _actor_pose.Rot().Euler();
//	ignition::math::Angle yaw_new = atan2( result_vel.X(), result_vel.Y() );
//	ignition::math::Angle yaw_delta = yaw_delta.HalfPi - yaw_new.Radian() - actor_rpy_initial.Z(); // HalfPi - theta(t=i+1) - theta(t=i)
//	yaw_delta.Normalize();

	ignition::math::Angle yaw_delta = atan2(result_vel.Y(), result_vel.X()) + 0.5*M_PI - actor_rpy_initial.Z();
	yaw_delta.Normalize();

#ifdef DEBUG_NEW_POSE
	std::cout << "\t rpy_initial: " << actor_rpy_initial;
//	std::cout << "  yaw_new: " << yaw_new;
	std::cout << "\t yaw_delta: " << yaw_delta;
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

	ignition::math::Angle yaw_new;
	yaw_new.Radian(actor_rpy_initial.Z() + yaw_delta.Radian() - 0.5*M_PI);

	// update yaw
//	yaw_new += yaw_delta;
	yaw_new.Normalize();

#ifdef DEBUG_NEW_POSE
	std::cout << "\t yaw_new: " << yaw_new;
#endif

	// calculate x and y velocity components based on new yaw angle (on-plane motions only)
	result_vel.X(+sin(yaw_new.Radian())*result_vel.SquaredLength());
	result_vel.Y(+cos(yaw_new.Radian())*result_vel.SquaredLength());

#ifdef DEBUG_NEW_POSE
	std::cout << "\n \t vel YAW-CORRECTED: " << result_vel;
#endif

	// calculate new pose - consider current pose, velocity and delta of time
	// set new rotation in the pose vector
	ignition::math::Vector3d new_pos;
	new_pos.X( _actor_pose.Pos().X() + result_vel.X()*_dt );
	new_pos.Y( _actor_pose.Pos().Y() + result_vel.Y()*_dt );
	new_pos.Z( _actor_pose.Pos().Z() + result_vel.Z()*_dt );

	ignition::math::Vector3d new_rot_rpy = _actor_pose.Rot().Euler();
	new_rot_rpy.X(1.5707);
	new_rot_rpy.Z(yaw_new.Radian());

	//
	// TODO: force pose.Z() according to current 'stance'
	//

	new_pose.Set(new_pos, new_rot_rpy);
	return new_pose;

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

	std::cout << std::endl;
	std::cout << "GetRelativeSpeed(): ";
	std::cout << "  obj_vel: " << _object_velocity;
	std::cout << "  act_vel: " << _actor_velocity;
	std::cout << "  v_rel:  " << rel_vel;
	std::cout << "  spd_rel: " << rel_vel.SquaredLength();
	std::cout << std::endl;

#endif

	return rel_vel.SquaredLength();;

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
	std::cout << "GetObjectsInteractionForce(): ";
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
		std::cout << "\t OBJECT TOO FAR AWAY, ZEROING FORCE! \t d_alpha_beta_length: " << d_alpha_beta_length;
		std::cout << std::endl;
		#endif

		return f_alpha_beta;
	}

	ignition::math::Angle actor_yaw(0.0);
	ignition::math::Angle object_yaw(0.0);
	uint8_t beta_rel_location = this->GetBetaRelativeLocation(actor_yaw, _d_alpha_beta);

	if ( beta_rel_location == SFM_BEHIND && d_alpha_beta_length > 0.5 ) {

		#ifdef DEBUG_INTERACTION_FORCE
		std::cout << "\t OBJECT FAR BEHIND, ZEROING FORCE!";
		std::cout << std::endl;
		#endif

		return f_alpha_beta;
	}

	double v_rel = GetRelativeSpeed(_actor_velocity, _object_velocity);

	if ( v_rel < 1e-12 ) {

		#ifdef DEBUG_INTERACTION_FORCE
		std::cout << "  v_rel = 0, ZEROING FORCE!";
		std::cout << std::endl;
		#endif

		return f_alpha_beta;

	}

	double theta_alpha_beta = this->GetAngleBetweenObjectsVelocities(_actor_pose, &actor_yaw, _object_pose, &object_yaw);
	ignition::math::Vector3d p_alpha = GetPerpendicularToNormal(_n_alpha, beta_rel_location); 	// actor's perpendicular (based on velocity vector)
	double exp_normal = ((-Bn * theta_alpha_beta * theta_alpha_beta)/v_rel) - Cn * d_alpha_beta_length;
	double exp_perpendicular = ((-Bp * theta_alpha_beta)/v_rel) - Cp * d_alpha_beta_length;
	f_alpha_beta = _n_alpha * An * exp(exp_normal) + p_alpha * Ap * exp(exp_perpendicular);

#ifdef DEBUG_INTERACTION_FORCE
	std::cout << "  v_rel: " << v_rel;
	std::cout << "  exp_n: " << exp_normal;
	std::cout << "  exp_p: " << exp_perpendicular;
	std::cout << "  f_alpha_beta: " << f_alpha_beta;
	std::cout << std::endl;
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
	std::cout << "GetAngleBetweenObjectsVelocities(): ";
#endif

#ifdef THETA_ALPHA_BETA_CONSIDER_ZERO_VELOCITY

	// check if objects are moving
	float actor_speed = _actor_vel.X()*_actor_vel.X() + _actor_vel.Y()*_actor_vel.Y();
	if ( actor_speed < 1e-3 ) {
#ifdef DEBUG_GEOMETRY_1
		std::cout << " ACTOR'S SPEED is close to 0! " << std::endl;
#endif
		return 0.0;
	}

	float object_speed = _object_vel.X()*_object_vel.X() + _object_vel.Y()*_object_vel.Y();
	if ( object_speed < 1e-3 ) {
#ifdef DEBUG_GEOMETRY_1
		std::cout << " OBJECT'S SPEED is close to 0! " << std::endl;
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
	std::cout << "\t yaw_actor: " << _actor_yaw->Radian() << "  yaw_object: " << _object_yaw->Radian() << "  yaw_diff: " << yaw_diff.Radian() << std::endl;
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
		std::cout << "GetPerpendicularToNormal(): " << "\t TOO SHORT! " << std::endl;
#endif

	}

#ifdef DEBUG_GEOMETRY_1
    std::cout << "GetPerpendicularToNormal(): " << "  x: " << p_alpha.X() << "  y: " << p_alpha.Y() << "  z: " << p_alpha.Z() << std::endl;
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
	angle_d_alpha_beta.Radian( atan2(d_alpha_beta_norm.X(), d_alpha_beta_norm.Y()) );
	angle_d_alpha_beta.Normalize();

	angle_relative = _actor_yaw + angle_d_alpha_beta;
	angle_relative.Normalize();

#ifdef DEBUG_GEOMETRY_2
	std::cout << "\t\t\t\t\t\t GetBetaRelativeLocation() " << "ACTOR yaw: " << _actor_yaw.Radian() << "  ANGLE d_alpha_beta: " << angle_d_alpha_beta.Radian() << "  ANGLE sum: " << angle_relative.Radian();
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
	std::cout << "  " << txt_dbg << std::endl;
#endif

	return rel_loc;

}

bool SocialForceModel::IsOutOfFOV(const double &_angle_relative) {

	if ( abs(_angle_relative) <= fov ) {
		return true;
	}

	return false;

}

// ------------------------------------------------------------------- //

SocialForceModel::~SocialForceModel() { }

}
