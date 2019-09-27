/*
 * SocialConductor.h
 *
 *  Created on: Aug 6, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_SOCIALCONDUCTOR_H_
#define INCLUDE_SFM_FUZZ_SOCIALCONDUCTOR_H_

#include <vector>
#include <ignition/math/Vector3.hh>
#include "SocialBehavioursDb.h"

namespace sfm {
namespace fuzz {

/// \brief This class takes care of calculations connected with summation
/// of consecutive social forces that are meant to be generated based on
/// the world configuration. The resulting social force is obtained
/// using superposition procedure (vector truncation possible if too long). TODO
class SocialConductor : public SocialBehavioursDb {

public:

	/// \brief Default constructor
	SocialConductor();

	/// \brief Updates `actual` social force vector stored internally.
	/// \param fuzz_output is an output (digital) of the defuzzification block
	/// \note DEPRECATED
	void apply(const double &fuzz_output);

	/// \brief Updates `actual` social force vector stored internally.
	/// \param term_name is an output (verbose) of the defuzzification block
	void apply(const std::string &term_name);

	/// \brief Returns the superposed social force vector.
	/// \return Superposed social force vector
	ignition::math::Vector3d getSocialVector() const;

	/// \brief Returns the last active behaviour ID
	/// \return
	uint8_t getBehaviourActive() const;

	/// \brief Resets the resulting vector.
	void reset();

	/// \brief Destructor
	virtual ~SocialConductor();

private:

	/// \brief Vector of social forces
	std::vector<ignition::math::Vector3d> sf_vector_;
	// TODO: std::vector<std::tuple<double, ignition::math::Vector3d> > // <fitness, force_vector>

	/// \brief Social force vector (the actual `social`;
	/// after superposition procedure)
	ignition::math::Vector3d sf_result_;

	/// \brief Stores last active behaviour
	uint8_t behaviour_active_;

	/// \brief Calculates the superposed vector according to the summed one
	/// considering truncation if its magnitude is too big.
	void superpose();

};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_SOCIALCONDUCTOR_H_ */
