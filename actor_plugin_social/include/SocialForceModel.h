/*
 * SocialForceModel.h
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#ifndef INCLUDE_SOCIALFORCEMODEL_H_
#define INCLUDE_SOCIALFORCEMODEL_H_

// #define SFM_HOMOGENOUS_POPULATION

#include <SocialForceModelUtils.h>
#include <ignition/math.hh> // ?? needed still

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

	ignition::math::Vector3d GetInternalAcceleration(const ignition::math::Pose3d &_actor_pose,
						  	  	  	  	    		 const ignition::math::Vector3d &_actor_vel,
													 const ignition::math::Pose3d &_actor_target,
													 ignition::math::Vector3d *_n_alpha);

	ignition::math::Vector3d GetInteractionComponent(const ignition::math::Pose3d &_actor_pose,
						  	  	  	  	    const ignition::math::Vector3d &_actor_vel,
											const ignition::math::Pose3d &_actor_target,
											const ignition::math::Vector3d &_n_alpha,
											const SFMObjectType &_object_type,
											const ignition::math::Pose3d &_object_pose,
											const ignition::math::Vector3d &_object_vel,
											const ignition::math::Box &_object_bb = ignition::math::Box()); // gazebo::math::Box is deprecated (Gazebo 8.0 and above)

	ignition::math::Pose3d GetNewPose(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_vel,
			const double &_dt,
			const ignition::math::Vector3d &_internal_acc,
			const std::vector<ignition::math::Vector3d> &_interactions_vector);

	virtual ~SocialForceModel();

private:

	void 	SetParameterValues(void);
/*
	ignition::math::Vector3d GetInternalAcceleration(const ignition::math::Pose3d &_actor_pose,
													 const ignition::math::Vector3d &_actor_vel,
													 const ignition::math::Pose3d &_actor_target);
*/
	ignition::math::Vector3d GetActorsInteraction(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Pose3d &_actor_target,
			const ignition::math::Vector3d &_n_alpha, 					// actor's normal (based on velocity vector)
			const ignition::math::Vector3d &_n_beta, 					// other actor's normal
			const ignition::math::Vector3d &_d_alpha_beta, 				// vector between objects poses
			const double &_v_rel);

	double GetVelocitiesAngle(const ignition::math::Vector3d &_n_alpha,
												const ignition::math::Vector3d &_n_beta,
												double *angle_alpha,
												double *angle_beta);

	uint8_t GetBetaRelativeLocation(
			const ignition::math::Vector3d &_n_alpha,
			const ignition::math::Vector3d &_d_alpha_beta);

			//const ignition::math::Pose3d &_actor_pose,
			//const ignition::math::Pose3d &_actor_target,
			//const ignition::math::Vector3d &_n_beta,
			//const double &_angle_alpha,
			//const double &_angle_beta


	ignition::math::Vector3d GetPerpendicularToNormal(
			const ignition::math::Vector3d &_n_alpha,
			const uint8_t &_beta_rel_location);


/*
	ignition::math::Vector3d GetStaticObjectInteraction(const ignition::math::Pose3d &_actor_pose,
												  	    const SFMObjectType &_type);
*/

	float relaxation_time;
	float speed_desired;
	float speed_max;
	float fov;
	float yaw_max_delta;

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
