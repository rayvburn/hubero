/*
 * TrapezoidParted.h
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_FUZZ_TRAPEZOIDPARTED_H_
#define INCLUDE_SFM_FUZZ_TRAPEZOIDPARTED_H_

#include <string>
#include <stdint.h>
#include <ignition/math/Angle.hh>
#include <vector>
#include <tuple>
#include <fl/term/Trapezoid.h>

namespace sfm {
namespace fuzz {

class TrapezoidParted {

public:

	/// @brief Constructor
	/// @param name: name of the trapezoid term
	/// @param intersection_deg
	TrapezoidParted(std::string name, double intersection_deg);

	/// @brief Updates both trapezoid parts
	/// @param start
	/// @param end
	/// @return
	/// @note `start` and `end` must be normalized angle values in radians
	bool update(const double &start, const double &end);

	/// @return Vector of pointers to fl::Trapezoid instances creating
	/// single region (may be wrapped around range's corners)
	std::vector<fl::Trapezoid*> getTrapezoids() const;

	/// Destructor
	virtual ~TrapezoidParted();

protected:



	std::string generateParams(const double &a, const double &b, const double &c, const double &d, const double &height = 1.0) const;

	double findHeight(const char& slope_type, const double &start, const double &end) const;

	void resetWrapped();

	/// Vector consisting of 1 or 2 elements, depending on the current
	/// location of corner points (gamma_cc etc.). Consists of 2 elements
	/// when range wraps from +PI to - PI.
	std::vector<fl::Trapezoid*> trapezoid_ptrs_;

	/// Defines how much a given region intersects the contiguous one (in `radians`)
	double intersection_;

};

} /* namespace fuzz */
} /* namespace sfm */

#endif /* INCLUDE_SFM_FUZZ_TRAPEZOIDPARTED_H_ */
