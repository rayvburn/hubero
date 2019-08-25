/*
 * TrapezoidLocIndep.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/TrapezoidLocIndep.h>

namespace sfm {
namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidLocIndep::TrapezoidLocIndep(std::string name, double intersection_deg, double length_deg)
		: TrapezoidParted::TrapezoidParted(name, intersection_deg), interval_(IGN_DTOR(length_deg)) {}

// ------------------------------------------------------------------- //

void TrapezoidLocIndep::configureTerm(const std::string &name, const ignition::math::Angle &gamma_center) {

	std::string params;
	// FIXME: both terms (wrapped too) may need to be changed
	std::tie(params, std::ignore) = calculateTrapezoid(gamma_center.Radian() - interval_,
													   gamma_center.Radian() + interval_,
													   false);

	this->trapezoid_ptrs_.at(0)->configure(params);

}

// ------------------------------------------------------------------- //

TrapezoidLocIndep::~TrapezoidLocIndep() {}

// ------------------------------------------------------------------- //
// private
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
