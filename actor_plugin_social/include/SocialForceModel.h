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
#include "BoundingCircle.h"
#include "CommonInfo.h"

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

// debug closest points
#include <vector>

// ---------------------------------

// 1 out of 2 or none of below possible

#define THETA_ALPHA_BETA_V2011	// "φ_αβ is an angle between velocity of pedestrian α and the displacement of pedestrian β"
//#define THETA_ALPHA_BETA_V2014		// "φ_αβ is the angle between n_α and d_αβ"

// ---------------------------------

// 1 out of 2 below possible

#define N_ALPHA_V2011		// "n_α is pointing in the opposite direction to the walking direction (deceleration force)"
//#define N_ALPHA_V2014		// "n_α is the direction of movement of pedestrian α"

// ---------------------------------

// #define PERPENDICULAR_COLLISION_AVOIDANCE_MOD // DEPRECATED?

// ---------------------------------

/*
 * At first it seemed that it's the bounding box calculation that makes algorithm unstable,
 * but without BB the situation is the same
 */

// #define BOUNDING_BOX_CALCULATION

#ifdef BOUNDING_BOX_CALCULATION
	#define BOUNDING_BOX_ONLY_FROM_OTHER_OBJECTS
	// #define BOUNDING_BOX_ALL_OBJECTS
#endif

#define BOUNDING_CIRCLE_CALCULATION	// bounding circle around actors only

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

// ---------------------------------

typedef enum {
	SFM_FRONT = 0,
	SFM_RIGHT_SIDE,
	SFM_LEFT_SIDE,
	SFM_BEHIND,
	SFM_UNKNOWN,
} RelativeLocation;

// ---------------------------------

class SocialForceModel {

public:

	SocialForceModel();


#ifdef BOUNDING_BOX_CALCULATION

// DEPRECATED
//	ignition::math::Line3d SocialForceModel::GetPossibleIntersectionLine(
//			const ignition::math::Pose3d &_actor_pose,
//			const ignition::math::Box &_bb,
//			const unsigned short int &_flag
//	);

	std::vector<ignition::math::Vector3d> CreateVerticesVector(
			const ignition::math::Box &_bb
	);

	std::vector<double> CalculateLengthToVertices(
			const ignition::math::Vector3d &_actor_pos,
			const std::vector<ignition::math::Vector3d> &_vertices_pts
	);

	ignition::math::Vector3d GetModelPointClosestToActor(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Box &_bb,
			const std::string &_model_name,				// debug only
			const ignition::math::Pose3d &_object_pose 	// debug only
	);

	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> GetActorModelBBsClosestPoints(
			const ignition::math::Pose3d &_actor_pose,
			//ignition::math::Pose3d *_actor_closest_to_model_pose,
			const ignition::math::Box &_actor_bb,
			const ignition::math::Pose3d &_object_pose,
			//ignition::math::Vector3d *_object_closest_to_actor_pos,
			const ignition::math::Box &_object_bb,
			const std::string &_object_name // debug only
	);

#endif

#ifdef BOUNDING_CIRCLE_CALCULATION

	ActorUtils::BoundingCircle GetActorBoundingCircle(
			const unsigned int _actor_id,
			const std::vector<ActorUtils::BoundingCircle> _actors_bounding_circles
	) const;

	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> GetActorModelBBsClosestPoints(
			const ignition::math::Pose3d &_actor_pose,
			const ActorUtils::BoundingCircle &_actor_bc,
			const ignition::math::Pose3d &_object_pose,
			const ignition::math::Box &_object_bb,
			const std::string &_object_name // debug only
			) const;

	std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> GetActorModelBBsClosestPoints(
			const ignition::math::Pose3d &_actor_pose,
			const ActorUtils::BoundingCircle &_actor_bc,
			const ignition::math::Pose3d &_object_pose,
			const ActorUtils::BoundingCircle &_object_bc,
			const std::string &_object_name // debug only
			) const;

#endif


	ignition::math::Angle GetYawMovementDirection(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_sf_vel);

	void Init(const unsigned short int _mass_person,
			  const float _desired_force_factor,
			  const float _interaction_force_factor,
				const gazebo::physics::WorldPtr _world_ptr);

	ignition::math::Vector3d GetInternalAcceleration(		const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Vector3d &_actor_target);

	inline double GetYawFromPose(const ignition::math::Pose3d &_pose);

	ignition::math::Vector3d GetInteractionComponent(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_vel,
			// const ignition::math::Pose3d &_actor_target,
			// const ignition::math::Vector3d &_n_alpha,
			// const SFMObjectType &_object_type,
			const ignition::math::Pose3d &_object_pose,
			const ignition::math::Vector3d &_object_vel,
			const ignition::math::Vector3d &_object_closest_point,
			//const ignition::math::Box &_object_bb = ignition::math::Box()); // gazebo::math::Box is deprecated (Gazebo 8.0 and above))
			const bool &_is_actor
	);

	ignition::math::Pose3d GetNewPose(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Pose3d &_actor_last_pose,
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Vector3d &_actor_target,
			const ignition::math::Vector3d &_social_force,
			const double &_dt,
			const uint8_t &_stance);

	ignition::math::Vector3d GetNormalToAlphaDirection(const ignition::math::Pose3d &_actor_pose);



#if		defined(THETA_ALPHA_BETA_V2011)

	double GetAngleBetweenObjectsVelocities(
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Angle &_actor_yaw,
			const ignition::math::Vector3d &_object_vel,
			const ignition::math::Angle &_object_yaw,
			const bool &_is_actor);


#elif	defined(THETA_ALPHA_BETA_V2014)
	double GetAngleAlphaBeta(
			const ignition::math::Vector3d &_n_alpha, 		// actor's normal (based on velocity vector)
			const ignition::math::Vector3d &_d_alpha_beta  	// vector between objects positions
	);

#else

	double GetAngleBetweenObjectsVelocities(
			const ignition::math::Pose3d &_actor_pose,
			ignition::math::Angle *_actor_yaw,
			const ignition::math::Pose3d &_object_pose,
			ignition::math::Angle *_object_yaw);

#endif



	RelativeLocation GetBetaRelativeLocation(
			const ignition::math::Angle &_actor_yaw,
			const ignition::math::Vector3d &_d_alpha_beta);

	bool IsOutOfFOV(const double &_angle_relative);

#if !defined(PERPENDICULAR_COLLISION_AVOIDANCE_MOD)
	ignition::math::Vector3d GetPerpendicularToNormal(
			const ignition::math::Vector3d &_n_alpha,
			const uint8_t &_beta_rel_location);
#else
	ignition::math::Vector3d GetPerpendicularToNormal(
			const ignition::math::Vector3d &_n_alpha,
			const uint8_t &_beta_rel_location,
			const ignition::math::Angle &_alpha_beta_angle);
#endif

	double GetRelativeSpeed(const ignition::math::Vector3d &_actor_velocity,
							const ignition::math::Vector3d &_object_velocity);

	bool IsActorFacingTheTarget(const ignition::math::Angle _yaw,
								const ignition::math::Vector3d _target);


	ignition::math::Vector3d GetObjectsInteractionForce(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_velocity,
			const ignition::math::Pose3d &_object_pose,
			const ignition::math::Vector3d &_object_velocity,
			const ignition::math::Vector3d &_n_alpha, 		// actor's normal (based on velocity vector)
			const ignition::math::Vector3d &_d_alpha_beta,
			//const ignition::math::Vector3d &_closest_point,
			const bool &_is_actor );

	ignition::math::Vector3d GetSocialForce(
		const gazebo::physics::WorldPtr _world_ptr,
		const std::string _actor_name,
		const ignition::math::Pose3d _actor_pose,
		const ignition::math::Vector3d _actor_velocity,
		const ignition::math::Vector3d _actor_target, // );
		const ActorUtils::CommonInfo &_actor_info);
//		const std::map<std::string, unsigned int>  _map_actor_name_id,
//		const std::vector<ignition::math::Vector3d> _actors_velocities,
//		const std::vector<ignition::math::Box> _actors_bounding_boxes);

	unsigned int GetActorID(const std::string _name, const std::map<std::string, unsigned int> _map);

	bool IsActor(const std::string &_name);

//	ignition::math::Vector3d GetActorVelocity(const gazebo::physics::ModelPtr &_model_ptr,
//			const std::map<std::string, unsigned int>  _map,
//			const std::vector<ignition::math::Vector3d> _actors_velocities);

	ignition::math::Vector3d GetActorVelocity(
			const unsigned int _actor_id,
			const std::vector<ignition::math::Vector3d> _actors_velocities
	);

	ignition::math::Box GetActorBoundingBox(
			const unsigned int _actor_id,
			const std::vector<ignition::math::Box> _actors_bounding_boxes
	);

	// debug closest points
//	std::vector<ignition::math::Pose3d> GetModelClosestPointsVector() const;
//	std::vector<ignition::math::Pose3d> GetActorClosestPointsVector() const;
	std::vector<ignition::math::Pose3d> GetClosestPointsVector() const;

	virtual ~SocialForceModel();

private:

	void 	SetParameterValues(void);

			//const ignition::math::Pose3d &_actor_pose,
			//const ignition::math::Pose3d &_actor_target,
			//const ignition::math::Vector3d &_n_beta,
			//const double &_angle_alpha,
			//const double &_angle_beta


	static constexpr double BOUNDING_BOX_Z_FIXED = 0.5; // on-plane objects considered

	// debug closest points
//	std::vector<ignition::math::Pose3d> models_closest_points;
//	std::vector<ignition::math::Pose3d> actor_closest_points;
	std::vector<ignition::math::Pose3d> closest_points;

	// stores previous location of a model relative to an actor
	std::map<std::string, RelativeLocation> map_models_rel_locations;

	float relaxation_time;
	float speed_desired;
	float speed_max;
	float force_max;
	float force_min;
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
