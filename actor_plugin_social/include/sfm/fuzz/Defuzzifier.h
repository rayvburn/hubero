/*
 * Defuzzifier.h
 *
 *  Created on: Apr 20, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_DEFUZZIFIER_H_
#define INCLUDE_SFM_FUZZ_DEFUZZIFIER_H_

#include "sfm/fuzz/Fuzzifier.h"

namespace sfm {
namespace fuzz {

class Defuzzifier {

public:

	/// \brief Default constructor
	Defuzzifier();

	/// \brief
	/// \note This must be called in each SFM calculation algorithm iteration (for each world object)
	/// \param f_alpha
	void setInternalForce(const ignition::math::Vector3d &f_alpha);

	/// \brief
	/// \note This must be called in each SFM calculation algorithm iteration (for each world object)
	/// \param f_alpha_beta_n
	void setInteractionForceNorm(const ignition::math::Vector3d &f_alpha_beta_n);

	/// \brief
	/// \note This must be called in each SFM calculation algorithm iteration (for each world object)
	/// \param f_alpha_beta_p
	void setInteractionForcePerp(const ignition::math::Vector3d &f_alpha_beta_p);

	/// \brief Sets fuzzifier's state, these 3 components act as input data for defuzzification.
	/// \param location
	/// \param direction
	/// \param level
	void setFuzzState(const FuzzLocation &location, const FuzzDirection &direction, const FuzzLevel &level);

	void setFuzzState(const std::tuple<FuzzLocation, FuzzDirection, FuzzLevel> &fuzz);

	/// \brief
	/// \return Social force component which needs to be aggregated while summing forces (new force type produced)
	ignition::math::Vector3d defuzzifySocialForce() const;

	///
	/// \brief Used to reset `slow_down` flag which is used to prevent multiple slow downs
	/// of the actor (may occur only when there are multiple actors present in the world).
	void reset();

	/// \brief Default destructor
	virtual ~Defuzzifier();

private:

	// ---------------------------------------------------------
	ignition::math::Vector3d passRight() const;
	ignition::math::Vector3d passLeft() const;
	ignition::math::Vector3d letPass() const;

	/// \brief Modifies perpendicular component of the `social force` vector (argument)
	/// so it points to the direction given by `dir`. Based on `level_` the vector
	/// is extended appropriately.
	/// \param dir
	/// \return Modified `social_force` vector
	ignition::math::Vector3d assertPerpDirection(const unsigned int &dir, bool slow_down = false) const;

	/// \brief Based on `level_` incorporates inverted `f_alpha` vector into `social_force`
	/// vector given as argument to this method.
	/// \return Modified `social_force` vector
	ignition::math::Vector3d weakenInternalForce() const;

	void assertPerpRight(ignition::math::Vector3d &social_force);
	void assertPerpLeft(ignition::math::Vector3d &social_force);
	// ---------------------------------------------------------

	/// \brief 2D rotation around Z-axis
	ignition::math::Vector3d rotateVector(const ignition::math::Vector3d &v, const double &angle) const;

	ignition::math::Vector3d f_alpha_;
	ignition::math::Vector3d f_alpha_beta_n_;
	ignition::math::Vector3d f_alpha_beta_p_;

	/// \brief Indicator whether actor has been slowed down in this iteration already.
	bool slowed_down_;

	sfm::fuzz::FuzzLocation location_;
	sfm::fuzz::FuzzDirection direction_;
	sfm::fuzz::FuzzLevel level_;

};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_DEFUZZIFIER_H_ */
