/*
 * SocialForceModel.h
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#ifndef INCLUDE_SOCIALFORCEMODEL_H_
#define INCLUDE_SOCIALFORCEMODEL_H_

// Gazebo
#include <actor/core/CommonInfo.h>
#include <actor/inflation/Border.h>
#include <actor/inflation/Box.h>				// cpp inclusion
#include <actor/inflation/Circle.h>				// cpp inclusion
#include <actor/inflation/Ellipse.h>			// cpp inclusion
#include <actor/ros_interface/ParamLoader.h>	// objects dictionary
#include <gazebo-8/gazebo/physics/World.hh>
#include <gazebo-8/gazebo/physics/Model.hh>

// ignition
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Angle.hh>
#include <sfm/core/ActorInfoDecoder.h>
#include <sfm/core/Inflator.h>
#include <sfm/core/ShiftRegister.h>
#include <sfm/core/WorldBoundary.h>

// C++ STL
#include <vector>	// closest points
#include <tuple>	// rel_loc

// Actor's data storage

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

// ---------------------------------

typedef enum {

	/** `The repulsive force from static obstacles f αi is modeled by using the functional
	 * form as given by the repulsive force for elliptical formulation` - 2014 */
	INTERACTION_ELLIPTICAL = 0,		// a.k.a. v2014

	/** `repulsion from walls uses the same formulas as the repulsion from other
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

/** 	φ_αβ issue
 * There is an inconsistency in papers connected with the Rudloff's version of Social Force model -
 * in Rudloff et al. 2011 - https://www.researchgate.net/publication/236149039_Can_Walking_Behavior_Be_Predicted_Analysis_of_Calibration_and_Fit_of_Pedestrian_Models
 * there is a statement that theta_alpha_beta is an "angle between velocity of pedestrian α and the displacement of pedestrian β"
 * whereas in Seer et al. 2014 - https://www.sciencedirect.com/science/article/pii/S2352146514001161
 * they say that in this model "φ αβ is the angle between n α and d αβ" (they call it phi instead of theta)
 */

/** 	n_α issue
 * Another inconsistency between 2011 and 2014 papers connected to Rudloff's SFM version is n_alpha issue.
 * In 2011 original paper there is said that n_alpha is "pointing in the opposite direction to the walking
 * direction (deceleration force)".
 * On the other hand in 2014 paper (that Rudloff is co-author of) they say: "n α is the direction of movement
 * of pedestrian α".
 */

typedef enum {

	/** " φ_αβ is an angle between velocity of pedestrian α and the displacement of pedestrian β "
	 * " n_α is pointing in the opposite direction to the walking direction (deceleration force) "
	 *   2011 */
	PARAMETER_DESCRIPTION_2011 = 0,

	/** " φ_αβ is the angle between n_α and d_αβ "
	 * " n_α is the direction of movement of pedestrian α "
	 *   2014 */
	PARAMETER_DESCRIPTION_2014,

	/** Connected only with another φ_αβ angle description:
	 * NOTE: below method of calculating the angle is only correct when both objects are:
	 * 		o dynamic,
	 * 		o currently moving,
	 * 		o already aligned with the to-target-direction,
	 * 		o there are no obstacles in the environment that will make the object not move along a straight line.
	 * NOT RECOMMENDED */
	PARAMETER_DESCRIPTION_UNKNOWN

} ParameterDescription;

// ---------------------------------

/** Related to an issue connected with a case when actor's
 * direction is opposite to the resulting force direction.
 * Most likely occurs when actor is forced by an other
 * object to turn out of the way pointing towards his
 * target. The force generated by the algorithm in such
 * situation is usually so strong that causes actor
 * 3/4 rotation first and then he keeps trying to reach
 * the goal.
 * Opposite force threshold is 0.85 * PI. */
typedef enum {

	/** Despite of the force pulling actor in the opposite direction
	  * (relative to the current movement direction) he will
	  * still try to go towards his goal. */
	OPPOSITE_FORCE_GO_TOWARDS_GOAL,

	/**
	  * The force is pulling the actor in the opposite direction but
	  * he tries to rotate left. This prevents force oscillation effect
	  * to cast down onto the actor's behaviour.
	  * Letting the raw force to lead the actor will most likely
	  * cause some 'collision' because oscillations from 1 side
	  * to the other will lock any stronger rotations
	  * and actor will accelerate going straight ahead
	  * (yes, opposite to the SFM algorithm result).
	  */
	OPPOSITE_FORCE_ROTATE_LEFT

} OppositeForceMethod;

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
	bool computeSocialForce(const gazebo::physics::WorldPtr &world_ptr,
			const ignition::math::Pose3d &actor_pose, const ignition::math::Vector3d &actor_velocity,
			const ignition::math::Vector3d &actor_target, const actor::core::CommonInfo &actor_info,
			const double &dt, const std::vector<std::string> &ignored_models_v);

	/// \brief Function which computes a new pose
	/// for an actor based on current one and the calculated
	/// social force
	ignition::math::Pose3d computeNewPose(const ignition::math::Pose3d &actor_pose, const ignition::math::Vector3d &actor_vel,
			const ignition::math::Vector3d &social_force, const ignition::math::Vector3d &target, const double &dt);

	/// \brief Resets class' internal state - mainly forces but resets internal classes too
	void reset();

	/// \brief Returns internal force vector
	ignition::math::Vector3d getForceInternal() const;
	/// \brief Returns interaction force vector
	ignition::math::Vector3d getForceInteraction() const;
	/// \brief Returns combined force vector (internal
	/// and interaction components summed up)
	ignition::math::Vector3d getForceCombined() const;

	/// \section Getter methods for data transferred
	/// to FuzzyProcessor and SocialConductor.
	/// Data acquisition is performed during a single
	/// Social Force calculation.
	/// All multi-element data are vectors of the same
	/// length whose corresponding elements are related
	/// to the same \beta object (i.e. i-th index of
	/// each vector variable is related to the same \beta
	/// object).
	/// \beta objects can be considered as obstacles,
	/// whereas getters from this section are related
	/// to dynamic obstacles only.
	///
	/// \brief Returns \f$\alpha\f$'s direction of motion
	/// expressed in world coordinate system
	double getDirectionAlpha() const;
	/// \brief Returns a vector of \beta objects direction
	/// of motion
	std::vector<double> getDirectionBetaDynamic() const;
	/// \brief Returns a vector of \beta objects' relative
	/// to \f$\alpha\f$ locations
	std::vector<double> getRelativeLocationDynamic() const;
	/// \brief Returns a set of dynamic objects vector
	/// directions. Each of these vectors connect
	/// \f$\alpha\f$ with \beta_i
	std::vector<double> getDistanceAngleDynamic() const;
	/// \brief Returns a set of lengths of vectors
	/// described in \ref getDistanceAngleDynamic
	std::vector<double> getDistanceDynamic() const;

	/// \brief Returns a distance to the closest
	/// static obstacle based on the world configuration
	/// valid in the last algorithm iteration.
	double getDistanceClosestStaticObstacle() const;
	/// \brief Returns a distance to the closest
	/// dynamic obstacle based on the world configuration
	/// valid in the last algorithm iteration.
	double getDistanceClosestDynamicObstacle() const;

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

	/// \brief Helper function which calculates a relative
	/// location of the investigated object based on actor's
	/// facing direction (in this case it is equal to a movement
	/// direction)
	/// \return A 3-element tuple consisting of:
	/// - relative location of the \beta object (relative to \f$\alpha\f$'s direction), see \ref RelativeLocation
	/// - relative location expressed as an angle (radians)
	/// - angle of the vector connecting \f$\alpha\f$ and \beta
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
	bool isModelNegligible(const std::string &model_name) const;

	/// \brief Checks whether a given object is listed in the `ignored models_v`
	/// dynamic set of objects.
	bool isModelNegligible(const std::string &model_name, const std::vector<std::string> ignored_models_v) const;

	/// \brief Evaluates whether an obstacle is dynamic or static
	bool isDynamicObstacle(const ignition::math::Vector3d &vel) const;

	/// \brief Rotates a given angle by +270/-90 degrees
	/// \param yaw_actor is the Yaw (rotation around the world Z axis) angle of the actor
	/// \return New orientation
	double convertActorToWorldOrientation(const double &yaw_actor) const;

	/// \brief Computes vector direction expressed as an angle in world coordinate system.
	/// \param v is a vector
	/// \return Angle
	double computeVectorDirection(const ignition::math::Vector3d &v) const;

	/// \brief Multiplies force components by a factor parameters
	/// and computes the combined force vector (summation).
	/// \note See \ref multiplyForces
	void factorInForceCoefficients();

	/// \brief Extends/truncates forces vectors if needed (i.e. combined force's
	/// magnitude too big / too small)
	/// \param dist_closest_static: distance to the closest static obstacle available
	/// \param dist_closest_dynamic: distance to the closest dynamic obstacle available
	void applyNonlinearOperations(const double &dist_closest_static, const double &dist_closest_dynamic);

	/// \brief Multiplies all force components (i.e. internal, interaction and social)
	/// by a given coefficient and updates the total force (`force_combined_`)
	/// according to a sum of components.
	/// \param coefficient: factor
	/// \note Truncates/extends all forces and calculates the combined one
	/// \note Non-linear operation
	void multiplyForces(const double &coefficient);

	/// \brief Updates the given distance variable (`dist_compare`)
	/// if a newly calculated distance (`dist`) is smaller.
	/// \param dist_compare: the smallest distance to an object so far
	/// \param dist: distance to the currently investigated object
	/// \return True if `dist_compare` variable has been updated
	bool updateClosestObstacleDistance(double &dist_compare, const double &dist) const;

	/// \brief Typical model is the one which stands for a valid Gazebo 3D model instance.
	/// World model (see parameter `world_dictionary/world_model`) is divided into 4 extra
	/// bounding boxes (only if the parameter is not empty, i.e. name is not set to `none`)
	bool isTypicalModel(const unsigned int &count_curr, const unsigned int &models_total);

	/// \brief Prepares `model_vel`, `model_raw_pose` and inflation shape according
	/// to the parameters and the Gazebo model setup.
	/// \return True if preprocessing finished with success
	/// \note non-const due to `setID` call of actor::core::CommonInfo
	bool preprocessTypicalModel(const gazebo::physics::WorldPtr &world_ptr, const size_t &model_num, const std::vector<std::string> &ignored_models_v,
			bool &is_an_actor, const actor::core::CommonInfo &actor_info, std::string &model_name,
			ignition::math::Vector3d &model_vel, ignition::math::Pose3d &model_raw_pose,
			actor::inflation::Border* &model_border_ptr);

	/// \brief Prepares `model_vel`, `model_raw_pose` and inflation rectangle according
	/// to the `world_dictionary/world_model/*` parameters.
	/// \return True if preprocessing finished with success
	bool preprocessWorldBoundary(const size_t &wall_num, bool &is_an_actor, std::string &model_name,
			ignition::math::Vector3d &model_vel, ignition::math::Pose3d &model_raw_pose,
			actor::inflation::Border* &model_border_ptr) const;

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

	/// \brief ShiftRegister is used for smoothing social force
	/// especially in close presence of the obstacles where most
	/// likely there is a highly weakened potential field.
	/// This in turn can cause a very difference force vector
	/// direction in 2 consecutive iterations.
	/// \note This saves only each `Arg2`-th sample and stores `Arg1` samples
	ShiftRegister<ignition::math::Vector3d> sf_values_{25, 15};

	/// \section Section: Force vectors section
	///
	/// \brief Internal force vector (a.k.a. f_alpha)
	ignition::math::Vector3d force_internal_;
	///
	/// \brief Interaction force vector created from
	/// a set of single interactions from static and dynamic
	/// obstacles (a.k.a sum of f_alpha_beta's)
	ignition::math::Vector3d force_interaction_;
	///
	/// \brief Result of summation of internal
	/// and interaction force vectors.
	ignition::math::Vector3d force_combined_;


	/// \section Section: Force factors
	///
	/// \brief Multiplier of the internal force
	float factor_force_internal_;
	///
	/// \brief Multiplier of the interaction force
	float factor_force_interaction_;


	/// \section Section: Social Force Model parameters
	///
	/// \brief Relaxation time
	float relaxation_time_;
	///
	/// \brief Desired speed
	float speed_desired_;
	///
	/// \brief Maximum allowable speed
	float speed_max_;
	///
	/// \brief Field of view
	float fov_;
	///
	/// \brief Mass of the person
	/// \note TODO: evaluate if it is helpful
	unsigned short int person_mass_;


	/// \section Section: Non-linear operations with force vectors parameters
	///
	/// \brief Maximum allowable force
	float force_max_;
	///
	/// \brief `Minimum` allowable force
	/// \note This may not be a minimum force,
	/// see \ref applyNonlinearOperations
	float force_min_;


	/// \section Section: SFM parameters
	///
	/// \brief Determines method of calculating force for static obstacles
	sfm::StaticObjectInteraction interaction_static_type_;
	/// \brief Determines type of inflation figure
	sfm::InflationType inflation_type_;
	/// \brief Related to an inconsistency in symbols
	/// presented in the papers, see \ref ParameterDescription
	sfm::ParameterDescription param_description_;
	/// \brief Method of computing a new pose while force
	/// of the opposite direction (relative to \f$\alpha\f$ is generated)
	sfm::OppositeForceMethod opposite_force_method_;
	/// \brief Determines maneuverability of the actor,
	/// for more details see \ref computeYawMovementDirection
	double factor_maneuverability_;
	/// \brief Defines whether interaction forces should be calculated;
	/// if set to false (parameter) will force actor to take the shortest
	/// possible path.
	bool disable_interaction_forces_;


	/// \brief Inflator class enlarges a surface around
	/// person to provide some safe distance from the obstacles
	sfm::Inflator inflator_;

	/// \brief ActorInfoDecoder is a helper class which helps
	/// during decoding of some ActorPlugin instances
	/// which cannot be saved in the WorldPtr.
	sfm::ActorInfoDecoder actor_decoder_;


	/// \section Section: Values transferred to the FuzzyProcessor and SocialConductor class instances.
	///
	/// \brief Current movement direction of an object
	double dir_alpha_;
	///
	/// \brief Direction of movement of the dynamic obstacles (a vector)
	std::vector<double> dir_beta_dynamic_v_;
	///
	/// \brief Relative (to the alpha) location of dynamic objects (a vector)
	std::vector<double> rel_loc_dynamic_v_;
	///
	/// \brief Angle of a distance vector connecting alpha and beta closest
	/// points (a vector)
	std::vector<double> dist_angle_dynamic_v_;
	///
	/// \brief Distance to the dynamic object (a vector)
	std::vector<double> dist_dynamic_v_;
	///
	/// \brief Stores distance to the closest dynamic obstacle
	double dist_closest_dynamic_;
	///
	/// \brief Stores distance to the closest static obstacle
	double dist_closest_static_;


	/// \brief Shared pointer to element of ParamLoader class.
	std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr_;

	/// \brief Stores bounding boxes of walls (only if `world_dictionary/world_model/name`
	/// is not set to `none`)
	sfm::core::WorldBoundary boundary_;

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

} /* namespace sfm */

#endif /* INCLUDE_SOCIALFORCEMODEL_H_ */
