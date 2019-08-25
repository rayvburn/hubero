/*
 * TrapezoidLocDep.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/TrapezoidLocDep.h>

namespace sfm {
namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidLocDep::TrapezoidLocDep(std::string name, double intersection_deg)
		: TrapezoidParted::TrapezoidParted(name, intersection_deg) {}

// ------------------------------------------------------------------- //

uint8_t TrapezoidLocDep::configureTerm(std::string name, const char &side,
			const ignition::math::Angle &gamma_start, const ignition::math::Angle &gamma_end) {

	std::string params;
	bool extra_term_needed;
//	uint8_t status = PROCESSOR_EXTRA_TERM_NONE;

	//
	std::string params_wrap;

	// based on `side` argument value, let's calculate trapezoid parameters
	// and decide whether an additional term is needed
	if ( side == 'r' ) {
		std::tie(params, extra_term_needed) = calculateTrapezoid(gamma_start.Radian(), gamma_end.Radian());
	} else {
		std::tie(params, extra_term_needed) = calculateTrapezoid(gamma_end.Radian(), gamma_start.Radian());
	}

	// update a term of the given `name`
	trapezoid_ptrs_.at(0)->configure(params);	// direction_.getTerm(name)->configure(params);

	// check whether an additional term is needed
//	if ( extra_term_needed ) {
	if ( params_wrap != std::string() ) {

		if ( side == 'r' ) {
			std::tie(params, std::ignore) = calculateTrapezoid(gamma_start.Radian(), gamma_end.Radian(), true);
		} else {
			std::tie(params, std::ignore) = calculateTrapezoid(gamma_end.Radian(), gamma_start.Radian(), true);
		}

		// save status value based on the `name`
		switch (name) {
		case("outwards"):
//				status |= PROCESSOR_EXTRA_TERM_OUTWARDS;
				break;
		case("cross_front"):
//				status |= PROCESSOR_EXTRA_TERM_CROSS_FRONT;
				break;
		case("cross_behind"):
//				status |= PROCESSOR_EXTRA_TERM_CROSS_BEHIND;
				break;
		}

//		// name of the term to check
//		name += "_extra";
//		if ( !direction_.hasTerm(name) ) {
//			direction_.addTerm(new fl::Trapezoid(name));
//		}
//		direction_.getTerm(name)->configure(params);
		trapezoid_ptrs_.at(1)->configure(params_wrap);

	} else {

		resetWrapped();

	}

//	return (status);
	return (1);

}

// ------------------------------------------------------------------- //

TrapezoidLocDep::~TrapezoidLocDep() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
