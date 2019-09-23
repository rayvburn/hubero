/*
 * Processor.h
 *
 *  Created on: Aug 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_PROCESSOR_H_
#define INCLUDE_SFM_FUZZ_PROCESSOR_H_

#include "fl/Headers.h"
#include <ignition/math/Angle.hh> // angle degree to radian conversion
#include <tuple>
#include <map>
#include "TrapezoidLocDep.h"
#include "TrapezoidLocIndep.h"

namespace sfm {
namespace fuzz {

class Processor {

public:
	Processor();

	void checkFl();

	void setDirectionAlpha(const double &dir_alpha);
	void setDirectionBeta(const double &dir_beta);

	/// \brief Setter method for `d_alpha_beta` angle (see SFM doc for details).
	/// `d_alpha_beta` angle is a direction of a vector connecting alpha and beta
	/// center positions.
	/// \param d_alpha_beta_angle is an angle described above
	void setDistanceAngle(const double &d_alpha_beta_angle);

	void setRelativeLocation(const double &rel_loc);

	/// \brief Executes fuzzy calculations. The `process()` call must be preceded by `updateRegions()`
	void process();

	/// \brief Based on the output variable's value, calculates the behavior needed to be applied.
	/// In case of 2 behaviors having non-zero memberships, their `strength` is different than 1.0
	/// (sums up to 1.0 though).
	/// \return Vector of tuples. Each tuple consists of behavior name (string) and membership level.
	std::vector<std::tuple<std::string, double> > getOutput() const;

	// TODO: check object dynamic
	/// \brief Currently investigated model's velocity vector
//	ignition::math::Vector3d vel_beta_;

//	void setVelsRelativeAngle(const double &vels_relative_angle);					// φ_αβ (FIXME: phi there...)
//	void setVelsRelativeAngle(const double &vel_alpha_angle, const double &vel_beta_angle);
//	void setObjectDirRelativeAngle(const double &object_dir_relative_angle);		// β position direction relative to α direction

	virtual ~Processor();

private:

	/// \brief Initializes fuzzy logic system (based on `fuzzylite` library)
	///  with values proper to this application.
	void init();

	// DEPRECATED?
//	bool rearrangeTerms(const std::string &name, const uint8_t &status_curr, uint8_t &status_global);
//	bool rearrangeTerms(const uint8_t &status_curr, uint8_t &status_global);

	/// \brief Determines location of the \beta element relative to \alpha direction of motion
	char decodeRelativeLocation(ignition::math::Angle eq, const ignition::math::Angle opp, ignition::math::Angle cc) const; // FIXME: DEPRECATED
	char decodeRelativeLocation() const;

	/// \brief Updates trapezoidal regions of input variables.
	/// \note Must be preceded by `setters` of input variables.
	void updateRegions();

	/// \brief Direction of the vector which connects \alpha 's position with \beta 's position.
	double d_alpha_beta_angle_;

	/// \brief Stores angle telling which \beta is located
	/// in terms of \alpha 's direction of motion.
	double rel_loc_;

	/// \brief Determines \alpha 's direction of motion.
	double alpha_dir_;

	/// \brief Determines \beta 's direction of motion.
	double beta_dir_;

	/// \brief Acts as a status register, in the .cpp there are few masks defined
	/// so every case possible can be recognized using just one 1-byte variable.
//	uint8_t term_extra_status_;

	/// \brief Array of indexes which `extra terms` are assigned to. Zero is not a valid number
	/// here (`extra term` with 0 index actually does not exist).
//	uint8_t term_extra_index_[3] = {0, 0, 0};

	/* ----- fuzzylite-related ----- */

	/// \brief Fuzzy logic engine
	fl::Engine engine_;

	/* ----- Input variables ----- */

	/// \brief An angle which helps specify on which hand-side the object is located relative to the actor
	fl::InputVariable location_; 	//	double object_dir_relative_angle_;

	// direction input variable section (consists of input variable and related trapezoidal terms)
	/// \brief A relative angle between 2 objects' velocities vectors; used to determine
	/// whether objects are moving in the same direction
	fl::InputVariable direction_; 		//	double vels_relative_angle_;

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction points outwards relative to the `alpha` direction
	TrapezoidLocDep trapezoid_out_{"outwards", 10};

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction crosses in front of the `alpha` center
	TrapezoidLocDep trapezoid_cf_{"cross_front", 10};

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction crosses behind the `alpha` center
	TrapezoidLocDep trapezoid_cb_{"cross_behind", 10};

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction points in the same direction as the `alpha`'s
	TrapezoidLocIndep trapezoid_eq_{"equal", 10, 5};

	/// \brief Trapezoidal term related to the case when `beta` object's
	/// direction points in the opposite direction as the `alpha`'s
	TrapezoidLocIndep trapezoid_opp_{"opposite", 10, 5};

	/* ----- Output variables ----- */
	fl::OutputVariable social_behavior_;
	std::string output_term_name_;
	double output_term_fitness_;

	/*  ----- Rule block ----- */
	fl::RuleBlock rule_block_;


//	/// \brief A distance between 2 objects; used to determine a level of condition
// //	double d_alpha_beta_len_;
//	fl::InputVariable d_alpha_beta_len_;


};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_PROCESSOR_H_ */
