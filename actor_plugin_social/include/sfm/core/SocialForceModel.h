/*
 * SocialForceModel.h
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#ifndef INCLUDE_SOCIALFORCEMODEL_H_
#define INCLUDE_SOCIALFORCEMODEL_H_

#include <SocialForceModelUtils.h>

#include "core/CommonInfo.h"

#include "inflation/Ellipse.h"
#include "inflation/Circle.h"
#include "inflation/Box.h"

#include <gazebo-8/gazebo/physics/World.hh>
#include <gazebo-8/gazebo/physics/Model.hh>
#include <ignition/math.hh> 		// is it still needed??

#include <sfm/core/Inflator.h>
#include <sfm/core/ActorInfoDecoder.h>

// debug closest points
#include <vector>


// ---------------------------------

// #define PERPENDICULAR_COLLISION_AVOIDANCE_MOD // DEPRECATED?

// ----------------------------------------------------------------------------------------------- //
/*
 * References:
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

namespace sfm {
namespace core {

// ---------------------------------

// #define INTERACTION_FORCE_STATIC_OBJ_V2011 	//
// #define INTERACTION_FORCE_STATIC_OBJ_V2014	//

typedef enum {

	/* `The repulsive force from static obstacles f αi is modeled by using the functional
	 * form as given by the repulsive force for elliptical formulation` - 2014 */
	INTERACTION_ELLIPTICAL = 0,		// a.k.a. v2014

	/* `repulsion from walls uses the same formulas as the repulsion from other
	 * pedestrians` - 2011 */
	INTERACTION_REPULSIVE_EVASIVE	// a.k.a. v2011

} StaticObjectInteraction;

// ---------------------------------

/*
 * At first it seemed that it's the bounding box calculation that makes algorithm unstable,
 * but without BB the situation is the same
 */

//#define BOUNDING_BOX_CALCULATION
#ifdef BOUNDING_BOX_CALCULATION
	//#define BOUNDING_BOX_ONLY_FROM_OTHER_OBJECTS
	//#define BOUNDING_BOX_ALL_OBJECTS
#endif
//#define BOUNDING_CIRCLE_CALCULATION	// bounding circle around actors only

										// compared to bounding box
//#define BOUNDING_ELLIPSE_CALCULATION

typedef enum {
	INFLATION_BOX_OTHER_OBJECTS = 0,
	INFLATION_BOX_ALL_OBJECTS,
	INFLATION_CIRCLE,					// BoundingCircle provides smoother force transitions while actor moves around obstacles COMPARED to BoundingBox
	INFLATION_ELLIPSE,
	INFLATION_NONE
} InflationType;

// ---------------------------------

typedef enum {
	LOCATION_FRONT = 0,
	LOCATION_RIGHT,
	LOCATION_LEFT,
	LOCATION_BEHIND,
	LOCATION_UNSPECIFIED
} RelativeLocation;

// ---------------------------------

//#define THETA_ALPHA_BETA_V2011
#define THETA_ALPHA_BETA_V2014		//

//#define N_ALPHA_V2011		//
#define N_ALPHA_V2014		//

/* 	φ_αβ
 * There is an inconsistency in papers connected with the Rudloff's version of Social Force model -
 * in Rudloff et al. 2011 - https://www.researchgate.net/publication/236149039_Can_Walking_Behavior_Be_Predicted_Analysis_of_Calibration_and_Fit_of_Pedestrian_Models
 * there is a statement that theta_alpha_beta is an "angle between velocity of pedestrian α and the displacement of pedestrian β"
 * whereas in Seer et al. 2014 - https://www.sciencedirect.com/science/article/pii/S2352146514001161
 * they say that in this model "φ αβ is the angle between n α and d αβ" (they call it phi instead of theta)
 */

/* 	n_α
 * Another inconsistency between 2011 and 2014 papers connected to Rudloff's SFM version is n_alpha issue.
 * In 2011 original paper there is said that n_alpha is "pointing in the opposite direction to the walking
 * direction (deceleration force)".
 * On the other hand in 2014 paper (that Rudloff is co-author of) they say: "n α is the direction of movement
 * of pedestrian α".
 */

typedef enum {

	/* " φ_αβ is an angle between velocity of pedestrian α and the displacement of pedestrian β "
	 * " n_α is pointing in the opposite direction to the walking direction (deceleration force) "
	 *   2011 */
	PARAMETER_DESCRIPTION_2011 = 0,

	/* " φ_αβ is the angle between n_α and d_αβ "
	 * " n_α is the direction of movement of pedestrian α "
	 *   2014 */
	PARAMETER_DESCRIPTION_2014,

	/* Connected only with another φ_αβ angle description:
	 * NOTE: below method of calculating the angle is only correct when both objects are:
	 * 		o dynamic,
	 * 		o currently moving,
	 * 		o already aligned with the to-target-direction,
	 * 		o there are no obstacles in the environment that will make the object not move along a straight line.
	 * NOT RECOMMENDED */
	PARAMETER_DESCRIPTION_UNKNOWN

} ParameterDescription;

// ---------------------------------

class SocialForceModel {

public:

	/// \brief Default constructor
	SocialForceModel();

	void Init(const unsigned short int _mass_person,
			  const float _desired_force_factor,
			  const float _interaction_force_factor,
			  const gazebo::physics::WorldPtr _world_ptr);

	/// \brief Function which calculates social force
	/// for an actor taking whole world's objects
	/// into consideration
	ignition::math::Vector3d computeSocialForce(const gazebo::physics::WorldPtr &world_ptr, const std::string &actor_name,
			const ignition::math::Pose3d &actor_pose, const ignition::math::Vector3d &actor_velocity,
			const ignition::math::Vector3d &actor_target, const actor::core::CommonInfo &actor_info,
			const double &dt);

	/// \brief Function which computes a new pose
	/// for an actor based on current one and the calculated
	/// social force
	ignition::math::Pose3d computeNewPose(const ignition::math::Pose3d &actor_pose, const ignition::math::Vector3d &actor_vel,
			const ignition::math::Vector3d &social_force, const double &dt);

	/// \brief Returns vector of poses of closest points between
	/// actor and other objects; makes use out of bounding
	/// boxes of world's objects and those boundings which
	/// had been created for actors
	std::vector<ignition::math::Pose3d> getClosestPointsVector() const;






	ignition::math::Angle GetYawMovementDirection(
			const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_vel,
			const ignition::math::Vector3d &_sf_vel);




	inline double GetYawFromPose(const ignition::math::Pose3d &_pose);





	ignition::math::Vector3d GetNormalToAlphaDirection(const ignition::math::Pose3d &_actor_pose);


	std::tuple<RelativeLocation, double> GetBetaRelativeLocation(
			const ignition::math::Angle &_actor_yaw,
			const ignition::math::Vector3d &_d_alpha_beta);

	bool IsOutOfFOV(const double &_angle_relative);


	ignition::math::Vector3d GetPerpendicularToNormal(
			const ignition::math::Vector3d &_n_alpha,
			const uint8_t &_beta_rel_location);

	// NEVER USED? DEPRECATED
#if defined(PERPENDICULAR_COLLISION_AVOIDANCE_MOD)
	ignition::math::Vector3d GetPerpendicularToNormal(
			const ignition::math::Vector3d &_n_alpha,
			const uint8_t &_beta_rel_location,
			const ignition::math::Angle &_alpha_beta_angle);
#endif

	double GetRelativeSpeed(const ignition::math::Vector3d &_actor_velocity,
							const ignition::math::Vector3d &_object_velocity);

	bool IsActorFacingTheTarget(const ignition::math::Angle _yaw,
								const ignition::math::Vector3d _target);






	/// \brief Default destructor
	virtual ~SocialForceModel();

private:

	/// \brief Helper function which assigns randomly
	/// generated numbers (with a proper mean and std. dev)
	/// to an algorithm parameters An, ... , Bw
	void setParameters();

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Social Force Model's components section (internal acceleration,
	// interaction force (dynamic/static) )

	/// \brief Helper function which calculates internal force
	/// term of an actor; this component describes a person's
	/// motivation to reach its current goal
	ignition::math::Vector3d computeInternalForce(const ignition::math::Pose3d &actor_pose, const ignition::math::Vector3d &actor_vel, const ignition::math::Vector3d &actor_target);

	/// \brief Helper function which calculates interaction
	/// force which another object (static or dynamic)
	/// exerts on the actor
	ignition::math::Vector3d computeInteractionForce(const ignition::math::Pose3d &actor_pose,
			const ignition::math::Vector3d &actor_vel, const ignition::math::Pose3d &object_pose,
			const ignition::math::Vector3d &object_vel, const bool &is_actor);

	ignition::math::Vector3d GetForceFromStaticObstacle(const ignition::math::Pose3d &_actor_pose,
			const ignition::math::Vector3d &_actor_velocity,
			const ignition::math::Pose3d &_object_pose,
			const double &_dt);
	// FIXME: dt needed to be passed!

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Section covering more of a geometry-related functions

	/// \brief Helper function which computes theta_αβ angle;
	/// fits 2011 configuration, where this angle is defined
	/// as: "an angle between velocity of pedestrian α and
	/// the displacement of pedestrian β"
	double computeThetaAlphaBetaAngle(const ignition::math::Vector3d &actor_vel, const ignition::math::Angle &actor_yaw,
									  const ignition::math::Vector3d &object_vel, const ignition::math::Angle &object_yaw,
									  const bool &is_actor);

	/// \brief Helper function which computes theta_αβ angle;
	/// fits 2014 configuration, where this angle is defined
	/// as: "an angle between n_α and d_αβ"
	/// \[param in] n_alpha - actor's normal (based on velocity
	/// vector)
	/// \[param in] d_alpha_beta - vector between objects
	/// positions
	double computeThetaAlphaBetaAngle(const ignition::math::Vector3d &n_alpha, const ignition::math::Vector3d &d_alpha_beta);

	/// \brief Helper function which computes theta_αβ angle;
	/// works only for dynamically moving actors -
	/// NOT RECOMMENDED
	double computeThetaAlphaBetaAngle(const ignition::math::Angle &actor_yaw, const ignition::math::Angle &object_yaw);





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


	sfm::core::StaticObjectInteraction interaction_static_type;
	sfm::core::InflationType inflation_type;
	sfm::core::ParameterDescription param_description_;


	sfm::core::Inflator inflate;
	sfm::core::ActorInfoDecoder actor_decoder;

	/* Model C -> Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014) */
	/* setting parameters static will create a population of actors moving in the same way */
	// TODO: make all actors share the same param values
	float An;
	float Bn;
	float Cn;
	float Ap;
	float Bp;
	float Cp;
	float Aw;
	float Bw;

};

} /* namespace core */
} /* namespace sfm */

#endif /* INCLUDE_SOCIALFORCEMODEL_H_ */
