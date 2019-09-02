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

bool TrapezoidLocIndep::update(const ignition::math::Angle &gamma_center) {

	// 2 angles must be created (normalization needed)
	ignition::math::Angle gamma_start = gamma_center;
	gamma_start.Radian(gamma_start.Radian() - interval_);
	gamma_start.Normalize();

	ignition::math::Angle gamma_end = gamma_center;
	gamma_end.Radian(gamma_end.Radian() + interval_);
	gamma_end.Normalize();
	return (TrapezoidParted::update(gamma_start.Radian(), gamma_end.Radian()));

}

//void TrapezoidLocIndep::configureTerm(const std::string &name, const ignition::math::Angle &gamma_center) {
//
//	std::string params;
//	// FIXME: both terms (wrapped too) may need to be changed
//	std::tie(params, std::ignore) = TrapezoidParted::calculateTrapezoid(gamma_center.Radian() - interval_,
//													   gamma_center.Radian() + interval_,
//													   false);
//
//	this->trapezoid_ptrs_.at(0)->configure(params);
//
//}

// ------------------------------------------------------------------- //

TrapezoidLocIndep::~TrapezoidLocIndep() {}

// ------------------------------------------------------------------- //
// private
// ------------------------------------------------------------------- //


// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
