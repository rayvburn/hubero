/*
 * SocialForceModel.h
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#ifndef INCLUDE_SOCIALFORCEMODEL_H_
#define INCLUDE_SOCIALFORCEMODEL_H_

// #define SFM_HOMOGENOUS_POPULATION
// #define DEBUG_TF

#include <SocialForceModelUtils.h>
// #include <gazebo/physics/World.hh>
#include <gazebo-8/gazebo/physics/World.hh>
#include <gazebo-8/gazebo/physics/Model.hh>
#include <ignition/math.hh> 		// is it still needed??

#ifdef DEBUG_TF
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#endif

// ----------------------------------------------------------------------------------------------- //
/* References:
 * 		- D. Helbing et al. 	- Social Force Model for Pedestrian Dynamics ‎(1998)
 * 		- Moussaid et al. 		- Experimental study of the behavioural mechanisms underlying
 * 		  						  self-organization in human crowds (2009)
 * 		- S. Seer et al. 		- Validating Social Force based Models with Comprehensive
 * 		  						  Real World Motion Data (2014)
 *
 * https://www.sciencedirect.com/science/article/pii/S2352146514001161
 * In the above paper there are results of the research presented. They point that
 * Model C (Rudloff et al. (2011) of SFM fits best the real world data.
 */
// ----------------------------------------------------------------------------------------------- //

namespace SocialForceModel {

class SocialForceModel {

public:

	SocialForceModel();

	void Init(const unsigned short int _mass_person,
			  const float _desired_force_factor,
			  const float _interaction_force_factor);

	ignition::math::Vector3d GetInternalAcceleration(		const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Vector3d &_actor_target);

	ignition::math::Vector3d GetInteractionComponent(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_vel,
			// const ignition::math::Pose3d &_actor_target,
			// const ignition::math::Vector3d &_n_alpha,
			// const SFMObjectType &_object_type,
			const ignition::math::Pose3d &_object_pose,
			const ignition::math::Vector3d &_object_vel
			//const ignition::math::Box &_object_bb = ignition::math::Box()); // gazebo::math::Box is deprecated (Gazebo 8.0 and above))
	);

	ignition::math::Pose3d GetNewPose(			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Vector3d &_social_force,
			const double &_dt,
			const uint8_t &_stance);

	ignition::math::Vector3d GetNormalToAlphaDirection(const ignition::math::Pose3d &_actor_pose);

	double GetAngleBetweenObjectsVelocities(
			const ignition::math::Pose3d &_actor_pose,
			ignition::math::Angle *_actor_yaw,
			const ignition::math::Pose3d &_object_pose,
			ignition::math::Angle *_object_yaw);

	uint8_t GetBetaRelativeLocation(
			const ignition::math::Angle &_actor_yaw,
			const ignition::math::Vector3d &_d_alpha_beta);

	bool IsOutOfFOV(const double &_angle_relative);

	ignition::math::Vector3d GetPerpendicularToNormal(
			const ignition::math::Vector3d &_n_alpha,
			const uint8_t &_beta_rel_location);


	double GetRelativeSpeed(const ignition::math::Vector3d &_actor_velocity,
							const ignition::math::Vector3d &_object_velocity);

	ignition::math::Vector3d GetObjectsInteractionForce(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_velocity,
			const ignition::math::Pose3d &_object_pose,
			const ignition::math::Vector3d &_object_velocity,
			const ignition::math::Vector3d &_n_alpha, 		// actor's normal (based on velocity vector)
			const ignition::math::Vector3d &_d_alpha_beta );

	ignition::math::Vector3d GetSocialForce(
		const gazebo::physics::WorldPtr _world_ptr,
		const std::string _actor_name,
		const ignition::math::Pose3d _actor_pose,
		const ignition::math::Vector3d _actor_velocity,
		const ignition::math::Vector3d _actor_target);


	virtual ~SocialForceModel();

private:

	void 	SetParameterValues(void);

			//const ignition::math::Pose3d &_actor_pose,
			//const ignition::math::Pose3d &_actor_target,
			//const ignition::math::Vector3d &_n_beta,
			//const double &_angle_alpha,
			//const double &_angle_beta


	float relaxation_time;
	float speed_desired;
	float speed_max;
	float fov;
	float yaw_max_delta;
	unsigned short int mass_person; 		// to evaluate if helpful
	float desired_force_factor;
	float interaction_force_factor;

#ifdef DEBUG_TF
	ignition::math::Pose3d actor_current_pose;	// needed to reference the tf to
	tf2_ros::TransformBroadcaster tf_broadcaster;
	geometry_msgs::TransformStamped tf_stamped;
	tf2::Matrix3x3 tf_mat_rot;
	tf2::Quaternion tf_quat;
#endif

#ifndef SFM_HOMOGENOUS_POPULATION

	/* Model C -> Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014) */
	float An;
	float Bn;
	float Cn;
	float Ap;
	float Bp;
	float Cp;
	float Aw;
	float Bw;

#else
	/* setting parameters static will create a population of actors moving in the same way */

	// TODO: make all actors share the same param values
	static float An;
	static float Bn;
	static float Cn;
	static float Ap;
	static float Bp;
	static float Cp;
	static float Aw;
	static float Bw;

#endif

};

}

#endif /* INCLUDE_SOCIALFORCEMODEL_H_ */
