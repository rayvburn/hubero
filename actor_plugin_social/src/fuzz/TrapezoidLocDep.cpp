/*
 * TrapezoidLocDep.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <fuzz/TrapezoidLocDep.h>

namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidLocDep::TrapezoidLocDep(std::string name, double intersection_deg)
		: TrapezoidParted::TrapezoidParted(name, intersection_deg) {}

// ------------------------------------------------------------------- //

bool TrapezoidLocDep::update(const char &side,
			const ignition::math::Angle &gamma_start, const ignition::math::Angle &gamma_end) {

	bool extra_term;

	// based on `side` argument value, let's calculate trapezoid parameters;
	// an additional term is configured internally if needed
	if ( side == 'r' ) {
		extra_term = TrapezoidParted::update(gamma_start.Radian(), gamma_end.Radian());
	} else if ( side == 'l' ) {
		extra_term = TrapezoidParted::update(gamma_end.Radian(), gamma_start.Radian());
	}

	return (extra_term);

}

// ------------------------------------------------------------------- //

TrapezoidLocDep::~TrapezoidLocDep() {
	// TODO Auto-generated destructor stub
}

// ------------------------------------------------------------------- //

// ------------------------------------------------------------------- //

} /* namespace fuzz */

