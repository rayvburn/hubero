/*
 * Fuzzifier.h
 *
 *  Created on: Apr 19, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_FUZZIFIER_H_
#define INCLUDE_SFM_FUZZ_FUZZIFIER_H_

#include <ignition/math/Vector3.hh>

namespace sfm {
namespace fuzz {

class Fuzzifier {

	/// \brief Human-readable environment state
	/// definition
	typedef enum {
		SFM_CONDITION_DYNAMIC_OBJECT_RIGHT = 0,
		SFM_CONDITION_FORCE_DRIVES_INTO_OBSTACLE,
		SFM_CONDITION_UNKNOWN = 100
	} SocialCondition;

	/// \brief Consecutive levels of detected
	/// condition's strength
	typedef enum {
		FUZZY_LEVEL_SMALL = 0,
		FUZZY_LEVEL_MEDIUM,
		FUZZY_LEVEL_HIGH,
		FUZZY_LEVEL_EXTREME,
		FUZZY_LEVEL_UNKNOWN
	} FuzzyLevel;

public:

	Fuzzifier();
	// void setParameters(const ignition::math::Vector3d &d_alpha_beta, const ignition::math::Vector3d &v_alpha, const ignition::math::Vector3d &v_beta, const ignition::math::Vector3d &social_force);

	void setDistanceVectorLength(const double &d_alpha_beta_len);
	void setToObjectDirectionRelativeAngle(const double &to_object_dir_relative_angle);
	void setVelocitiesRelativeAngle(const double &vels_relative_angle);

	/// \brief // pass a vector, calculate direction internally
	void setSocialForce(const ignition::math::Vector3d &social_force);

	bool isConditionDetected();

	/// \brief Resets parameters, need to be
	// invoked after each iteration
	void resetParameters();

	// defuzzifier - std::tuple<double, ignition::math::Vector3d, ignition::math::Vector3d>
	virtual ~Fuzzifier();

private:

	/// \brief Flags indicating that a corresponding parameter
	/// had been set since last resetParameters() procedure
	bool dist_set_;
	bool dir_angle_set_;
	bool vels_angle_set_;
	bool sf_set_;

	/// \brief A relative angle between 2 objects' velocities
	/// vectors; used to determine whether objects are moving
	/// in the same direction
	double vels_relative_angle_;

	/// \brief A distance between 2 objects; used to determine
	/// a level of condition
	double dist_between_objects_;

	/// \brief An angle which helps specify on which hand-side
	/// the object is located relative to actor
	double to_object_dir_relative_angle_;

	/// \brief An angle which SF vector's direction represents;
	/// used to determine whether force drives an actor straight
	/// into obstacle;
	double sf_vector_dir_angle_;

	/// \brief A variable storing current condition
	SocialCondition condition_;

	/// \brief A variable which points a level of a detected
	/// `social condition`
	FuzzyLevel level_;

};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_FUZZIFIER_H_ */
