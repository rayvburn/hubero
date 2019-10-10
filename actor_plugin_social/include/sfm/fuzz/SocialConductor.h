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

	/// \note DEPRECATED, use \ref apply instead
	/// \brief Updates `actual` social force vector stored internally.
	/// \param fuzz_output is an output (digital) of the defuzzification block
	//
	/// \brief Updates `actual` social force vector stored internally.
	/// \param term_name is an output (verbose) of the defuzzification block
	//
	void apply(	const ignition::math::Vector3d &force_combined,	const double &dir_alpha,
				const std::vector<double> &dist_v, const std::vector<std::tuple<std::string, double> > &fuzz_output_v);

	/// \brief Returns the superposed social force vector.
	/// \return Superposed social force vector
	ignition::math::Vector3d getSocialVector() const;

	/// \brief Returns the last active behaviour expressed in verbal way
	/// \return
	std::string getBehaviourActive() const;

	/// \brief Destructor
	virtual ~SocialConductor();

private:

//	/// \brief Vector of social forces
//	std::vector<ignition::math::Vector3d> sf_vector_;

	/// \brief Social force vector (the actual `social`;
	/// after superposition procedure)
	ignition::math::Vector3d sf_result_;

	/// \brief Stores active behaviour list as a string (each activated
	/// behaviour is placed in a new line).
	std::string behaviour_active_str_;

	/// \brief Calculates the superposed vector according to the summed one
	/// considering truncation if its magnitude is too big.
	void superpose(const std::vector<ignition::math::Vector3d> &forces);

	/// \brief Updates `behaviour_active_str_` according to its length
	/// and given behaviour name (method argument)
	void updateActiveBehaviour(const std::string &beh_name);

};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_SOCIALCONDUCTOR_H_ */
