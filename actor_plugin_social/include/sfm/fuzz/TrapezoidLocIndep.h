/*
 * TrapezoidLocIndep.h
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_TRAPEZOIDLOCINDEP_H_
#define INCLUDE_SFM_FUZZ_TRAPEZOIDLOCINDEP_H_

#include "TrapezoidParted.h"

namespace sfm {
namespace fuzz {

class TrapezoidLocIndep : public TrapezoidParted {

public:

	/// @param name
	/// @param intersection_deg
	/// @param length_deg: region in fact has 0 length, so it must be artificially extended
	/// to actually be taken into consideration during reasoning
	TrapezoidLocIndep(std::string name, double intersection_deg, double length_deg);

	/// @param name
	/// @param gamma_center
	void configureTerm(const std::string &name, const ignition::math::Angle &gamma_center);

	virtual ~TrapezoidLocIndep();

private:

	double interval_;

};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_TRAPEZOIDLOCINDEP_H_ */
