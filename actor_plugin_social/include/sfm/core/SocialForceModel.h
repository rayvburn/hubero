/*
 * SocialForceModel.h
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#ifndef INCLUDE_SOCIALFORCEMODEL_H_
#define INCLUDE_SOCIALFORCEMODEL_H_

// Gazebo
#include <gazebo-8/gazebo/physics/World.hh>
#include <gazebo-8/gazebo/physics/Model.hh>

// ignition
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Angle.hh>

// C++ STL
#include <vector>	// closest points
#include <tuple>	// rel_loc

// Actor's data storage
#include "core/CommonInfo.h"

// Helper
#include <ros_interface/ParamLoader.h>	// objects dictionary

// Bounding models of an actor
#include "inflation/Ellipse.h"
#include "inflation/Circle.h"
#include "inflation/Box.h"

// SFM's other files
#include <sfm/core/Inflator.h>
#include <sfm/core/ActorInfoDecoder.h>
#include <sfm/core/ShiftRegister.h>
#include <sfm/fuzz/Fuzzifier.h>
#include <sfm/fuzz/Defuzzifier.h>

// Processor - checking fuzzylite behavior
#include <sfm/fuzz/Processor.h>
#include <sfm/fuzz/SocialConductor.h>

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

typedef enum {

	/* `The repulsive force from static obstacles f αi is modeled by using the functional
	 * form as given by the repulsive force for elliptical formulation` - 2014 */
	INTERACTION_ELLIPTICAL = 0,		// a.k.a. v2014

	/* `repulsion from walls uses the same formulas as the repulsion from other
	 * pedestrians` - 2011 */
	INTERACTION_REPULSIVE_EVASIVE	// a.k.a. v2011

} StaticObjectInteraction;

// ---------------------------------

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

/* 	φ_αβ issue
 * There is an inconsistency in papers connected with the Rudloff's version of Social Force model -
 * in Rudloff et al. 2011 - https://www.researchgate.net/publication/236149039_Can_Walking_Behavior_Be_Predicted_Analysis_of_Calibration_and_Fit_of_Pedestrian_Models
 * there is a statement that theta_alpha_beta is an "angle between velocity of pedestrian α and the displacement of pedestrian β"
 * whereas in Seer et al. 2014 - https://www.sciencedirect.com/science/article/pii/S2352146514001161
 * they say that in this model "φ αβ is the angle between n α and d αβ" (they call it phi instead of theta)
 */

/* 	n_α issue
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

	/// \brief Function which sets internal parameters according to loaded parameters
	void init(std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr,
			  const InflationType &inflation_type, const std::string &actor_name,
			  const gazebo::physics::WorldPtr &world_ptr);

	/// \brief Function which calculates social force
	/// for an actor taking whole world's objects
	/// into consideration
	ignition::math::Vector3d computeSocialForce(const gazebo::physics::WorldPtr &world_ptr,
			const ignition::math::Pose3d &actor_pose, const ignition::math::Vector3d &actor_velocity,
			const ignition::math::Vector3d &actor_target, const actor::core::CommonInfo &actor_info, const double &dt);

	/// \brief Function which computes a new pose
	/// for an actor based on current one and the calculated
	/// social force
	ignition::math::Pose3d computeNewPose(const ignition::math::Pose3d &actor_pose, const ignition::math::Vector3d &actor_vel,
			const ignition::math::Vector3d &social_force, const ignition::math::Vector3d &target, const double &dt);

	/// \brief Returns vector of poses of closest points between
	/// actor and other objects; makes use out of bounding
	/// boxes of world's objects and those boundings which
	/// had been created for actors
	std::vector<ignition::math::Pose3d> getClosestPointsVector() const;

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

	/// \brief Helper function which calculates the internal force
	/// term of an actor; this component describes a person's
	/// motivation to reach its current goal
	ignition::math::Vector3d computeInternalForce(const ignition::math::Pose3d &actor_pose, const ignition::math::Vector3d &actor_vel, const ignition::math::Vector3d &actor_target);

	/// \brief Helper function which calculates interaction
	/// force which another object (static or dynamic)
	/// exerts on the actor (tuple's first element).
	/// Additionally, a distance vector to the currently
	/// considered obstacle is returned as the tuple's second
	/// element. Third element is a length of that vector.
	std::tuple<ignition::math::Vector3d, ignition::math::Vector3d, double> computeInteractionForce(const ignition::math::Pose3d &actor_pose,
			const ignition::math::Vector3d &actor_vel, const ignition::math::Pose3d &object_pose,
			const ignition::math::Vector3d &object_vel, const bool &is_actor);

	/// \brief Helper function which computes a repulsive
	/// force which static obstacle exerts on the actor;
	/// fits 2014 configuration and used only in this case.
	/// \return 1st element - force vector
	/// 		2nd element - distance vector (between actor and obstacle)
	/// 		3rd element - distance vector's length
	std::tuple<ignition::math::Vector3d, ignition::math::Vector3d, double> computeForceStaticObstacle(const ignition::math::Pose3d &actor_pose,
			const ignition::math::Vector3d &actor_velocity, const ignition::math::Pose3d &object_pose,
			const double &dt);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Section covering more of a geometry-related functions
	/* Theta alpha-beta calculation */

	/// \brief Helper function which computes theta_αβ angle;
	/// fits 2011 configuration, where this angle is defined
	/// as: "an angle between velocity of pedestrian α and
	/// the displacement of pedestrian β"
	double computeThetaAlphaBetaAngle(const ignition::math::Vector3d &actor_vel, const ignition::math::Angle &actor_yaw,
									  const ignition::math::Vector3d &object_vel, const ignition::math::Angle &object_yaw,
									  const ignition::math::Vector3d &d_alpha_beta, const bool &is_actor);

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

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	/* Resultative vector components calculation - normal (n_alpha)
	 * and perpendicular (p_alpha) */

	/// \brief Helper function which computes a vector
	/// that is normal to alpha's direction vector;
	/// depending on the `parameter description` set
	/// the vector is calculated differently
	ignition::math::Vector3d computeNormalAlphaDirection(const ignition::math::Pose3d &actor_pose);

	/// \brief Helper function which takes a given n_alpha
	/// vector and based on currently investigated object's
	/// relative to the actor location calculates a perpendicular
	/// to n_alpha vector
	ignition::math::Vector3d computePerpendicularToNormal(const ignition::math::Vector3d &n_alpha,
			const RelativeLocation &beta_rel_location);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	/* General functions section (still geometry-related) */

	// FIXME: return tuple description
	/// \brief Helper function which calculates a relative
	/// location of the investigated object based on actor's
	/// facing direction (in this case it is equal to a movement
	/// direction)
	/// \return A tuple of relative location of the object
	/// and an angle between actor's movement and object's
	/// location
	std::tuple<RelativeLocation, double, double> computeObjectRelativeLocation(const ignition::math::Angle &actor_yaw,
			const ignition::math::Vector3d &d_alpha_beta);

	/// \brief Helper function which checks whether a given
	/// angle is within actor's field of view bounds
	inline bool isOutOfFOV(const double &angle_relative);

	/// \brief Helper function which pulls out a yaw angle
	/// from a given pose; at the moment it is just a wrapper
	/// on a Pose's class method but it could be handy
	/// when an additional angle transformation should be used
	/// NOTE: actor's coordinate system is rotated 90 deg CW
	/// compared to the world's one
	inline double getYawFromPose(const ignition::math::Pose3d &pose);

	/// \brief Helper function which calculates relative
	/// speed based on 2 given velocity vectors
	double computeRelativeSpeed(const ignition::math::Vector3d &actor_vel, const ignition::math::Vector3d &object_vel);

	/// \brief Helper function which is used in computeNewPose;
	/// based on a velocity vector which would be a result
	/// from a raw SFM algorithm it refines actor's yaw
	/// to prevent immediate rotations;
	/// the bigger the actor's velocity is the smaller
	/// rotation is allowed (modeled with a decreasing
	/// exponential function);
	/// it has a big impact on actors' behavior
	ignition::math::Angle computeYawMovementDirection(const ignition::math::Pose3d &actor_pose,
			const ignition::math::Vector3d &actor_vel, const ignition::math::Vector3d &sf_vel,
			const ignition::math::Vector3d &target);

	/// \brief Checks whether a given object is listed in the `ignored models` set of objects
	/// called dictionary here. Uses actor::core::Target static function as helper.
	bool isModelNegligible(const std::string &model_name);

	/// \brief Rotates a given angle by +270/-90 degrees
	/// \param yaw_actor is the Yaw (rotation around the world Z axis) angle of the actor
	/// \return New orientation
	double convertActorToWorldOrientation(const double &yaw_actor) const;

	/// \brief Computes vector direction expressed as an angle in world coordinate system.
	/// \param v is a vector
	/// \return Angle
	double computeVectorDirection(const ignition::math::Vector3d &v) const;

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	/// \brief A vector of poses of closest points
	/// between actor's and object's boundings
	std::vector<ignition::math::Pose3d> closest_points_;

	/// \brief Name of the actor owning SFM instance
	std::string owner_name_;

	/// \brief A map which stores previous location
	/// of a model relative to an actor;
	/// a historical data is used when relative location
	/// value oscillates from one side to the other
	std::map<std::string, RelativeLocation> map_models_rel_locations_;

	// TODO: save only 1/100 samples instead of 50 consecutive ones
	ShiftRegister<ignition::math::Vector3d> sf_values_{25, 15};

	/// \brief Social Force Model parameters variables
	float relaxation_time_;
	float speed_desired_;
	float speed_max_;

	float fov_;
	unsigned short int person_mass_; 		// to evaluate if helpful
	float internal_force_factor_;
	float interaction_force_factor_;

	float force_max_;
	float force_min_;


	sfm::core::StaticObjectInteraction interaction_static_type_;
	sfm::core::InflationType inflation_type_;
	sfm::core::ParameterDescription param_description_;


	sfm::core::Inflator inflator_;
	sfm::core::ActorInfoDecoder actor_decoder_;

	sfm::fuzz::Fuzzifier fuzz_;
	sfm::fuzz::Defuzzifier defuzz_;

	//
	sfm::fuzz::Processor fuzzy_processor_;
	sfm::fuzz::SocialConductor social_conductor_;

	/// @brief Shared pointer to element of ParamLoader class.
	std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr_;

	/* Model C -> Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014) */
	/* setting parameters static will create a population of actors moving in the same way */
	// TODO: make all actors share the same param values
	float An_;
	float Bn_;
	float Cn_;
	float Ap_;
	float Bp_;
	float Cp_;
	float Aw_;
	float Bw_;

};

} /* namespace core */
} /* namespace sfm */

#endif /* INCLUDE_SOCIALFORCEMODEL_H_ */
