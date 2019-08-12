/*
 * Fuzzifier.h
 *
 *  Created on: Apr 19, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_FUZZIFIER_H_
#define INCLUDE_SFM_FUZZ_FUZZIFIER_H_

#include <ignition/math/Vector3.hh>
#include <tuple>

namespace sfm {
namespace fuzz {

/* NOTE: when put into class the error appears:
 * libsocial_force_model.so: undefined symbol: _ZNK3sfm4fuzz9Fuzzifier26getSocialConditionAndLevelEv */

///// \brief Human-readable environment state definition
//typedef enum {
//	FUZZ_OBJECT_RIGHT_DIR_EQUAL = 0u,  	//!< FUZZ_OBJECT_RIGHT_DIR_EQUAL
//	FUZZ_OBJECT_RIGHT_DIR_PERP_OUT,    	//!< FUZZ_OBJECT_RIGHT_DIR_PERP_OUT
//	FUZZ_OBJECT_RIGHT_DIR_PERP_CROSS,  	//!< FUZZ_OBJECT_RIGHT_DIR_PERP_CROSS
//	FUZZ_OBJECT_RIGHT_DIR_OPPOSITE,    	//!< FUZZ_OBJECT_RIGHT_DIR_OPPOSITE
//	FUZZ_OBJECT_RIGHT_DIR_UNKNOWN,     	//!< FUZZ_OBJECT_RIGHT_DIR_UNKNOWN
//	FUZZ_OBJECT_LEFT,                  	//!< FUZZ_OBJECT_LEFT
//	FUZZ_OBJECT_LOCATION_UNKNOWN		//!< FUZZ_OBJECT_LOCATION_UNKNOWN
//} FuzzLocation;

/// \brief Human-readable environment state definition
typedef enum {
	FUZZ_LOCATION_RIGHT = 0u,//!< FUZZ_LOCATION_RIGHT
	FUZZ_LOCATION_LEFT,      //!< FUZZ_LOCATION_LEFT
	FUZZ_LOCATION_FRONT,     //!< FUZZ_LOCATION_FRONT
	FUZZ_LOCATION_BACK,      //!< FUZZ_LOCATION_BACK
	FUZZ_LOCATION_UNKNOWN    //!< FUZZ_LOCATION_UNKNOWN
} FuzzLocation;

/// \brief TODO
typedef enum {
	FUZZ_DIR_EQUAL = 0u,//!< FUZZ_DIR_EQUAL
	FUZZ_DIR_OPPOSITE,  //!< FUZZ_DIR_OPPOSITE
	FUZZ_DIR_PERP_OUT,  //!< FUZZ_DIR_PERP_OUT
	FUZZ_DIR_PERP_CROSS,//!< FUZZ_DIR_PERP_CROSS
	FUZZ_DIR_UNKNOWN    //!< FUZZ_DIR_UNKNOWN
} FuzzDirection;

/// \brief Consecutive levels of detected condition strength
typedef enum {
	FUZZY_LEVEL_LOW = 0,	//!< FUZZY_LEVEL_LOW
	FUZZY_LEVEL_MEDIUM, 	//!< FUZZY_LEVEL_MEDIUM
	FUZZY_LEVEL_HIGH,   	//!< FUZZY_LEVEL_HIGH
	FUZZY_LEVEL_EXTREME,	//!< FUZZY_LEVEL_EXTREME
	FUZZY_LEVEL_NONE 		//!< FUZZY_LEVEL_NONE
} FuzzLevel;


/// \brief Social Force Model's (which in fact does not really implement any social behavior,
/// except passing on the right) extension implementing some basic behaviors
/// related to the right-side `traffic` rules.
class Fuzzifier {

public:

	/// \brief Default constructor
	Fuzzifier();

	/// \brief Fuzzifier's parameters setters (straight-forward)
//	void setDistanceVectorLength(const double &d_alpha_beta_len);
//	void setToObjectDirectionRelativeAngle(const double &to_object_dir_relative_angle);
//	void setVelocitiesRelativeAngle(const double &vels_relative_angle);
//	void setOtherObjectVelocity(const ignition::math::Vector3d &object_vel);

	//

//	void setInternalForce(const ignition::math::Vector3d &f_alpha);
//	void setInteractionForceNorm(const ignition::math::Vector3d &f_alpha_beta_n);
//	void setInteractionForcePerp(const ignition::math::Vector3d &f_alpha_beta_p);

	// to check whether static or dynamic, static can be omitted!
	void setObjectVelocity(const ignition::math::Vector3d &vel_beta);				// angle(v_β)
	void setObjectVelocity(const double &yaw_beta, const double &speed);			// angle(v_β)
	void setVelsRelativeAngle(const double &vels_relative_angle);					// φ_αβ (FIXME: phi there...)
	void setVelsRelativeAngle(const double &vel_alpha_angle, const double &vel_beta_angle);
	void setObjectDirRelativeAngle(const double &object_dir_relative_angle);		// β angle relative to α
	void setDistanceVectorLength(const double &d_alpha_beta_len);					// ||d_αβ||
	//

//	/// \brief Setter to which a social force vector is passed
//	/// but its direction angle is computed internally and saved
//	/// for later use
//	void setSocialForce(const ignition::math::Vector3d &social_force);

	/// \brief Checks whether some condition is detected
	/// based on previously given parameter values
	bool isApplicable();

	/// \brief Resets parameters, need to be invoked
	/// after each iteration
	void resetParameters();

	std::tuple<FuzzLocation, FuzzDirection, FuzzLevel> getFuzzyState() const;

	/// \brief Default destructor
	virtual ~Fuzzifier();

private:

	/// \brief Currently investigated model could be a static
	/// or a dynamic one - thus a velocity vector's length
	/// check is performed in this helper function
	inline bool isDynamicObject();

	void computeFuzzLocation();
	void computeFuzzDirection();
	void computeFuzzLevel();

	/// \brief Checks if a predicate is in active state -
	/// performs multiple `bool` flags check
	bool isActivePredicateObjectRight();

	/// \brief Checks if a predicate is in active state -
	/// performs multiple `bool` flags check
	bool isActivePredicateForceDirection();

//	/// \brief Flags indicating that a corresponding parameter
//	/// had been set since last resetParameters() procedure
//	bool vel_object_set_;
//	bool vels_angle_set_;
//	bool dist_set_;
//	bool dir_angle_set_;
//	bool sf_set_;

	// FIXME: ^ check velocity vector length instead


	/* ----------- Algorithm input data section ----------- */
//	/// \brief A vector driving the actor towards the current goal location.
//	ignition::math::Vector3d f_alpha_;
//
//	/// \brief A vector expressing interaction force exerted by `beta` on `alpha`.
//	/// Stores only the `normal` component.
//	ignition::math::Vector3d f_alpha_beta_n_;
//
//	/// \brief A vector expressing interaction force exerted by `beta` on `alpha`.
//	/// Stores only the `perpendicular` component.
//	ignition::math::Vector3d f_alpha_beta_p_;

	/* Input variables */
	/// \brief Currently investigated model's velocity vector
	ignition::math::Vector3d vel_beta_;

	/// \brief A relative angle between 2 objects' velocities vectors; used to determine
	/// whether objects are moving in the same direction
	double vels_relative_angle_;

	/// \brief An angle which helps specify on which hand-side the object is located relative to the actor
	double object_dir_relative_angle_;

	/// \brief A distance between 2 objects; used to determine a level of condition
	double d_alpha_beta_len_;

	/* ----------- Algorithm output data section ----------- */
	/// \brief A variable storing current location (of Beta relative to Alpha)
	FuzzLocation location_;

	/// \brief Beta's motion direction (based on velocity) relative to Alpha
	FuzzDirection direction_;

	/// \brief A variable which points a level of a detected `social condition`
	FuzzLevel level_;

};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_FUZZIFIER_H_ */
