/*
 * TrapezoidLocDep.h
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_TRAPEZOIDLOCDEP_H_
#define INCLUDE_SFM_FUZZ_TRAPEZOIDLOCDEP_H_

#include "TrapezoidParted.h"

namespace sfm {
namespace fuzz {

class TrapezoidLocDep : public TrapezoidParted {

public:

	TrapezoidLocDep(std::string name, double intersection_deg);

	/// @brief TODO: some image needed to illustrate the situation
	/// @param name
	/// @param side
	/// @param gamma_start
	/// @param gamma_end
	/// @return
	bool update(const char &side, const ignition::math::Angle &gamma_start, const ignition::math::Angle &gamma_end);

	virtual ~TrapezoidLocDep();

private:


};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_TRAPEZOIDLOCDEP_H_ */
