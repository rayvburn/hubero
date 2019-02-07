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

// ------------------------------------------------------------------- //

SocialForceModel::SocialForceModel():

	fov(1.40), speed_max(1.50), yaw_max_delta(20.0 / M_PI * 180.0), mass_person(80) {

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

ignition::math::Vector3d SocialForceModel::GetInternalAcceleration(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_vel,
		const ignition::math::Pose3d &_actor_target)
{

	// internal acceleration is a first step in SF calculations, n_alpha is useful later on and in this stage

	// TODO: a lot of allocations here, could it be solved?
	ignition::math::Vector3d to_goal_vector = (_actor_target.Pos() - _actor_pose.Pos());
	ignition::math::Vector3d to_goal_direction = to_goal_vector.Normalize();
	ignition::math::Vector3d ideal_vel_vector = speed_desired * to_goal_direction;
	ignition::math::Vector3d f_alpha = mass_person * (1/relaxation_time) * (ideal_vel_vector - _actor_vel);

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
		const ignition::math::Pose3d &_actor_target,
		const ignition::math::Vector3d &_n_alpha,
		const SFMObjectType &_object_type,
		const ignition::math::Pose3d &_object_pose,
		const ignition::math::Vector3d &_object_vel,
		const ignition::math::Box &_object_bb)

{

	// calculations for beta object
	ignition::math::Vector3d d_alpha_beta = (_object_pose.Pos() - _actor_pose.Pos());
	ignition::math::Vector3d n_beta = _object_vel;
	n_beta.Normalize();
	double v_rel = _actor_vel.SquaredLength() + _object_vel.SquaredLength();

	ignition::math::Vector3d f_alpha_beta = GetActorsInteraction(_actor_pose, _actor_target, _n_alpha, n_beta, d_alpha_beta, v_rel);

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
			const double &_dt,
			const ignition::math::Vector3d &_internal_acc,
			const std::vector<ignition::math::Vector3d> &_interactions_vector)
{

	ignition::math::Pose3d new_pose;
	ignition::math::Vector3d interaction_sum = {0.0F , 0.0F , 0.0F};

	for ( uint i = 0; i < _interactions_vector.size(); i++ ) {
		// get the cumulative vector expressed as a sum of single interactions
		interaction_sum += _interactions_vector[i];
	}

	ignition::math::Vector3d result_acc = _internal_acc + interaction_sum;
	ignition::math::Vector3d result_vel = _actor_vel + result_acc * _dt;

	if ( result_vel.Length() > speed_max ) {
		/* if calculated speed is bigger than max speed -> perform normalization
		// leave velocity direction as is, shorten the vector to max possible */
		result_vel = result_vel.Normalize() * speed_max;
	}

	/* Now consider the yaw angle of an actor - it is crucial to make him face his
	 * current movement direction */
	ignition::math::Vector3d actor_rpy_initial = _actor_pose.Rot().Euler();
	ignition::math::Angle yaw_new = atan2( result_vel.X(), result_vel.Y() );
	ignition::math::Angle yaw_delta = M_PI/2.0F - yaw_new.Radian() - actor_rpy_initial.Z(); // HalfPi - theta(t=i+1) - theta(t=i)
	yaw_delta.Normalize();

	// avoid big yaw changes, force smooth rotations; truncate to max
	if ( abs( yaw_delta.Radian() ) > yaw_max_delta ) {
		(yaw_delta < 0) ? (yaw_delta = -yaw_max_delta) : (yaw_delta = +yaw_max_delta);
	}

	// calculate x and y velocity components based on new yaw angle

	// set new rotation in the pose vector

	// force pose.Z() according to current 'stance'


	// new_pose = _actor_pose

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

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d SocialForceModel::GetActorsInteraction(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Pose3d &_actor_target,
		const ignition::math::Vector3d &_n_alpha, 		// actor's normal (based on velocity vector)
		const ignition::math::Vector3d &_n_beta, 		// other actor's normal
		const ignition::math::Vector3d &_d_alpha_beta, 	// vector between objects poses
		const double &_v_rel)							// relative speed
{

	// TODO: only 6 closest actors taken into consideration?
	ignition::math::Vector3d f_alpha_beta = {0.0F , 0.0F , 0.0F};

	/* Force is zeroed in 2 cases:
	 * 	- distance between actors is bigger than 5 meters
	 * 	- the other actor is more than 0.5 m behind */
	double d_alpha_beta_length = _d_alpha_beta.Length();
	if ( d_alpha_beta_length > 5.0 ) {
		return f_alpha_beta;
	}

	uint8_t beta_rel_location = GetBetaRelativeLocation(_n_alpha, _d_alpha_beta); // beta location relative to alpha, FOV considered

	if ( beta_rel_location == SFM_BEHIND && d_alpha_beta_length > 0.5 ) {
		return f_alpha_beta;
	}

	double angle_alpha, angle_beta = 0.00F;
	double theta_alpha_beta = GetVelocitiesAngle(_n_alpha, _n_beta, &angle_alpha, &angle_beta); // angle between velocity of pedestrian α and the displacement of pedestrian β

	ignition::math::Vector3d p_alpha = GetPerpendicularToNormal(_n_alpha, beta_rel_location); 	// actor's perpendicular (based on velocity vector)
	double exp_normal = ((-Bn * theta_alpha_beta * theta_alpha_beta)/_v_rel) - Cn * d_alpha_beta_length;
	double exp_perpendicular = ((-Bp * theta_alpha_beta)/_v_rel) - Cp * d_alpha_beta_length;
	f_alpha_beta = _n_alpha * An * exp(exp_normal) + p_alpha * Ap * exp(exp_perpendicular);

	return f_alpha_beta;

}

// ------------------------------------------------------------------- //

// dynamic objects interaction
double SocialForceModel::GetAngleBetweenObjectsVelocities(
		const ignition::math::Pose3d &_actor_pose,
		const ignition::math::Vector3d &_actor_vel,
		ignition::math::Angle *_actor_yaw,					// alpha's yaw - will be calculated and saved for further work
		const ignition::math::Pose3d &_object_pose,
		const ignition::math::Vector3d &_object_vel,
		ignition::math::Angle *_object_yaw)					// beta's yaw - will be calculated and saved for further work
{

	// only on-plane movement considered

	std::cout << "GetAngleBetweenObjectsVelocities(): ";

#ifdef THETA_ALPHA_BETA_CONSIDER_ZERO_VELOCITY

	// check if objects are moving
	float actor_speed = _actor_vel.X()*_actor_vel.X() + _actor_vel.Y()*_actor_vel.Y();
	if ( actor_speed < 1e-3 ) {
		std::cout << " ACTOR'S SPEED is close to 0! " << std::endl;
		return 0.0;
	}

	float object_speed = _object_vel.X()*_object_vel.X() + _object_vel.Y()*_object_vel.Y();
	if ( object_speed < 1e-3 ) {
		std::cout << " OBJECT'S SPEED is close to 0! " << std::endl;
		return 0.0;
	}

	//actor_speed  = sqrt(actor_speed);
	//object_speed = sqrt(object_speed);

	// velocities are not 0 so calculate the angle between those 2 vectors

#endif

	ignition::math::Vector3d rpy_actor = _actor_pose.Rot().Euler();
	ignition::math::Vector3d rpy_object = _object_pose.Rot().Euler();

	/*
	ignition::math::Angle yaw_actor;
	ignition::math::Angle yaw_object;

	yaw_actor.Radian(rpy_actor.Z());
	yaw_actor.Normalize();

	yaw_object.Radian(rpy_object.Z());
	yaw_object.Normalize();

	ignition::math::Angle yaw_diff = yaw_object - yaw_actor;
	yaw_diff.Normalize();

	std::cout << "\t yaw_actor: " << yaw_actor.Radian() << "  yaw_object: " << yaw_object.Radian() << "  yaw_diff: " << yaw_diff.Radian() << std::endl;

	return yaw_diff.Radian();
	*/

	_actor_yaw->Radian(rpy_actor.Z());
	_actor_yaw->Normalize();

	_object_yaw->Radian(rpy_object.Z());
	_object_yaw->Normalize();

	ignition::math::Angle yaw_diff = *_object_yaw - *_actor_yaw;
	yaw_diff.Normalize();

	std::cout << "\t yaw_actor: " << _actor_yaw->Radian() << "  yaw_object: " << _object_yaw->Radian() << "  yaw_diff: " << yaw_diff.Radian() << std::endl;

	return yaw_diff.Radian();

}

// interaction between dynamic and static object

double SocialForceModel::GetVelocitiesAngle(const ignition::math::Vector3d &_n_alpha,
											const ignition::math::Vector3d &_n_beta,
											double *angle_alpha,
											double *angle_beta)

{

	*angle_alpha = atan2(_n_alpha.X(), _n_alpha.Y());
	*angle_beta  = atan2(_n_beta.X(),  _n_beta.Y());
	ignition::math::Angle phi_alpha_beta = *angle_alpha - *angle_beta;
	phi_alpha_beta.Normalize();
	return (phi_alpha_beta.Radian());

}

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
		std::cout << "GetPerpendicularToNormal(): " << "\t TOO SHORT! " << std::endl;

	}

    std::cout << "GetPerpendicularToNormal(): " << "  x: " << p_alpha.X() << "  y: " << p_alpha.Y() << "  z: " << p_alpha.Z() << std::endl;

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

	std::cout << "\t\t\t\t\t GetBetaRelativeLocation() " << "ACTOR yaw: " << _actor_yaw.Radian() << "  ANGLE d_alpha_beta: " << angle_d_alpha_beta.Radian() << "  ANGLE sum: " << angle_relative.Radian();
	std::string txt_dbg;

	// consider FOV
	if ( IsOutOfFOV(angle_relative.Radian() ) ) {
		rel_loc = SFM_BEHIND;
		txt_dbg = "BEHIND";
	} else if ( angle_relative.Radian() <= 0.0 ) {
		rel_loc = SFM_RIGHT_SIDE;
		txt_dbg = "RIGHT";
	} else if ( angle_relative.Radian() > 0.0 ) {
		rel_loc = SFM_LEFT_SIDE;
		txt_dbg = "LEFT";
	}

	std::cout << "  " << txt_dbg << std::endl;

	return rel_loc;

}

bool SocialForceModel::IsOutOfFOV(const double &_angle_relative) {

	if ( abs(_angle_relative) <= fov ) {
		return true;
	}

	return false;

}

uint8_t SocialForceModel::GetBetaRelativeLocation(
		const ignition::math::Vector3d &_n_alpha,
		const ignition::math::Vector3d &_d_alpha_beta)
{

	/* Normalize d_alpha_beta to calculate the angle between n_alpha
	 * and d_alpha_beta - this will tell on which side beta is relative
	 * to alpha */

	uint8_t rel_loc = 0;

	ignition::math::Vector3d d_alpha_beta_norm = _d_alpha_beta;
	d_alpha_beta_norm.Normalize();

	/*
	// https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
	double dot_product = _n_alpha.X()*d_alpha_beta_norm.X() + _n_alpha.Y()*d_alpha_beta_norm.Y(); 	// dot = x1*x2 + y1*y2
	double determinant = _n_alpha.X()*d_alpha_beta_norm.Y() - _n_alpha.Y()*d_alpha_beta_norm.X(); 	// det = x1*y2 - y1*x2
	ignition::math::Angle angle = atan2(determinant, dot_product); 												// angle = atan2(det, dot)
	angle.Normalize();

	// angle bigger than FOV -> out of sight -> beta is behind the current actor (alpha)
	if ( abs(angle.Radian()) > fov) {
		rel_loc = SFM_BEHIND;
		return rel_loc;
	}

	// TODO: debugging needed here, sign is very important
	// assuming angle is pointing FROM d_alpha_beta TO n_alpha
	if ( angle.Radian() >= 0.0F ) {
		rel_loc = SFM_RIGHT_SIDE;
	} else {
		rel_loc = SFM_LEFT_SIDE;
	}

	return rel_loc;
	*/

	return rel_loc;

}

// ------------------------------------------------------------------- //

SocialForceModel::~SocialForceModel() { }

}
