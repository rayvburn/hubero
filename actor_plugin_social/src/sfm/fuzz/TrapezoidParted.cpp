/*
 * TrapezoidParted.cpp
 *
 *  Created on: Aug 23, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/TrapezoidParted.h>
#include <math.h> // fabs()

namespace sfm {
namespace fuzz {

// ------------------------------------------------------------------- //

TrapezoidParted::TrapezoidParted(std::string name, double intersection_deg)
		: intersection_(IGN_DTOR(intersection_deg))
{
	// create 2 trapezoid instances, rules will need both of them for deduction;
	// both trapezoids are first initialized with NaNs so they will not be recognized
	// as valid regions until become fully initialized
	trapezoid_ptrs_.push_back(new fl::Trapezoid(name));
	trapezoid_ptrs_.push_back(new fl::Trapezoid(name)); // .append("_wrap") // 2 terms of the same name are allowed
}

// ------------------------------------------------------------------- //

TrapezoidParted::~TrapezoidParted() {
	delete trapezoid_ptrs_.at(0);
	delete trapezoid_ptrs_.at(1);
}

// ************************************************************************************************************
// private section
// ************************************************************************************************************

bool TrapezoidParted::update(const double &start, const double &end) {

	// Creates parameter configuration equal to the uninitialized fl::Trapezoid instance
	std::string params("nan nan nan nan");
	std::string params_wrap("nan nan nan nan");

	// Variable to detect whether term (trapezoid) has been `wrapped`
	bool status = false;

	// helper variables to perform initial tests
	ignition::math::Angle a(start - intersection_);	a.Normalize(); // A is the first  vertex of the trapezoid
	ignition::math::Angle d(end + intersection_);	d.Normalize(); // D is the fourth vertex of the trapezoid

	// check whether trapezoid's A vertex is located before (smaller than) the D vertex
	if ( a.Radian() < d.Radian() ) {

		// if below condition is met then full term is located within allowable bounds (in <-PI; +PI> range)
		if ( a.Radian() >= (-IGN_PI) && d.Radian() <= (+IGN_PI) ) {

			// normal operation - only single term's parameters must be found
//			params += a.Radian(); 	params += " ";														// A
//			gamma.Radian(start);	gamma.Normalize();		params += gamma.Radian(); 	params += " ";	// B
//			gamma.Radian(end);		gamma.Normalize();		params += gamma.Radian(); 	params += " ";	// C
//			params += d.Radian();	params += " ";														// D
//			params += "1.0";																			// height
			params = generateParams(a.Radian(), start, end, d.Radian(), 1.0);

		}

	} else {

		// both terms' parameters must be found

		// NOTE: `start` is equal to the B vertex
		// NOTE: `end`   is equal to the C vertex

		// find interval which contains +PI argument - it can be achieved
		// via simple comparison of the 2 consecutive vertices locations
		if ( a.Radian() >= start ) {

			// CASE 1: trapezoid's first part finishes somewhere between A and B vertices
			//
			// NOTE: fuzzylite does not complain about terms which are too wide for
			// the allowable range - calculates the membership functions just right
			// as long as the input variable does not exceed the given range.
			// In the other words: terms' bounds can exceed <-PI; +PI> range
			// as long as the `INPUT VARIABLE` is always withing those bounds.

			// firstly, find how much the B vertex goes out of +PI range (i.e. the value before normalization)
			double wrap = std::fabs(-IGN_PI - start);

			// argument B vertex would have if range would not be limited
			double b_out_range = IGN_PI + wrap;     // positive side case

			// secondly, calculate the height for the IGN_PI argument
			// TODO: height not need to be cut?
//			double height = findHeight('a', a.Radian(), b_out_range);

			// first part's configuration (range artificially extended by 5 degrees)
			params = generateParams(a.Radian(), b_out_range, b_out_range + IGN_DTOR(5.0), b_out_range + IGN_DTOR(5.0), 1.0);

			// consider negative side case now (wrap from the positive side to the negative one)
			wrap = std::fabs(IGN_PI - start); // `wrap2` in notebook
			double a_out_range = -IGN_PI - wrap;

			// second part's configuration
			params_wrap = generateParams(a_out_range, start, end, d.Radian(), 1.0);



		} else if ( start >= end ) {

			// CASE 2: trapezoid's first part finishes somewhere between B and C vertices
			params = generateParams(a.Radian(), start, +IGN_PI, +IGN_PI, 1.0);

			// find the distance which has been cut off (`end` is surely negative)
			double cutoff = std::fabs(-IGN_PI - end); // TODO: is this needed?

			// find the second part's configuration
//			params_wrap = generateParams(-IGN_PI, -IGN_PI, cutoff, d.Radian(), 1.0); // TODO: ?
			params_wrap = generateParams(-IGN_PI, -IGN_PI, end, d.Radian(), 1.0);



		} else if ( end >= d.Radian() ) {

			// CASE 3: trapezoid's first part finishes somewhere between C and D vertices
			//
			// firstly, find how much the D vertex goes out of +PI range (i.e. the value before normalization)
			double wrap = std::fabs(-IGN_PI - d.Radian());

			// argument D vertex would have if range would not be limited
			double d_out_range = IGN_PI + wrap;     // positive side case

			// secondly, calculate the height for the IGN_PI argument FIXME? needed?
			// TODO: height not need to be cut?
//			double height = findHeight('a', a.Radian(), d_out_range);

			// first part's configuration (range artificially extended by 5 degree)
			params = generateParams(a.Radian(), start, end, d_out_range, 1.0);

			// consider negative side case now (wrap from the positive side to the negative one)
			//
			// difference between (+IGN_PI) and C vertex location (end)
			wrap = std::fabs(IGN_PI - end); // `wrap2` in the notebook
			double c_out_range = -IGN_PI - wrap;

			// second part's configuration
			params_wrap = generateParams(c_out_range - IGN_DTOR(5.0), c_out_range - IGN_DTOR(5.0), c_out_range, d.Radian(), 1.0);

		}
		status = true;



	}

	//
	/*
	// GAP shifts region bounding point towards the region's center
	static const double GAP = IGN_DTOR(5.0);
	// INTERSECTION is an interval used to artificially extend the region's range
	static const double INTERSECTION = IGN_DTOR(15.0);

	// allocate variables
	ignition::math::Angle gamma;
	std::string params;
	std::string params_wrap;

	// check whether `start` angle is smaller than `end` angle
	if ( start <= end ) {

		// normal operation
		gamma.Radian(start - intersection_);	gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(start);					gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(end);						gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(end + intersection_);		gamma.Normalize();		params += gamma.Radian();


	} else {

		// NOTE: Seems that `start` is bigger than `end` so a `breakdown` of ranges occurred.
		// This kind of situation must be handled differently - see `fuzzylite`'s
		// `Trapezoid` `membership` method:
		// https://fuzzylite.github.io/fuzzylite/d0/d26/classfl_1_1_trapezoid.html#a266a40979ac36f6012efce935302e983
		// 2 separate trapezoids must be created (first with c = d and a != b,
		// second with a = b and c != d)
		gamma.Radian(end + GAP - INTERSECTION);	gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(end + GAP);				gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(IGN_PI);					gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(IGN_PI);					gamma.Normalize();		params += gamma.Radian();
		// FIXME: intersection range must moved to the opposite side of the x-axis
		// when <end; pi> range is too narrow
		return (std::make_tuple(params, true));


		// compute the second part of the trapezoid
		gamma.Radian(-IGN_PI);					gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(-IGN_PI);					gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(end + GAP - INTERSECTION);	gamma.Normalize();		params += gamma.Radian(); 	params += " ";
		gamma.Radian(end + GAP);				gamma.Normalize();		params += gamma.Radian();
		// FIXME: consider `intersection` movement from the close-to-pi range
		return (std::make_tuple(params, false));

	}
	*/
	//

	// NOTE: params_wrap may be not modified
//	return (std::make_tuple(params, params_wrap));

	// if `wrap` did not occur, let's try to reset the second part (with index of 1)
	if ( !status ) {
		// sets all fields to `nan` except for `height`
		// TODO: is this needed? see below
		resetWrapped();
	}

	// update both trapezoids with new parameters
	trapezoid_ptrs_.at(0)->configure(params);
	trapezoid_ptrs_.at(1)->configure(params_wrap);

	return (status);

}

// ------------------------------------------------------------------- //

std::vector<fl::Trapezoid*> TrapezoidParted::getTrapezoids() const {
	return (trapezoid_ptrs_);
}

// ------------------------------------------------------------------- //

std::string TrapezoidParted::generateParams(const double &a, const double &b, const double &c,
		const double &d, const double &height) const {

	std::string params("");

	params += std::to_string(a); 	params += " ";		// A
	params += std::to_string(b); 	params += " ";		// B
	params += std::to_string(c); 	params += " ";		// C
	params += std::to_string(d);	params += " ";		// D
	params += std::to_string(height);					// height

	return (params);

}

// ------------------------------------------------------------------- //

double TrapezoidParted::findHeight(const char& slope_type, const double &start, const double &end) const {


	double y = std::numeric_limits<double>::max();

	if ( slope_type == 'a') {

		// ascending - looking for Y-value for the +PI argument
		y = (IGN_PI - start)/(end - start);

	} else if ( slope_type == 'd' ) {

		// descending - looking for Y-value for the -PI argument
		y = (end - (-IGN_PI))/(end - start);

	}

	return (y);

}

// ------------------------------------------------------------------- //

void TrapezoidParted::resetWrapped() {

	// check first vertex of the `wrapped` trapezoid,
	// if it is equal to some valid number let's set
	// all `vertices` to NaNs and `height` to 0
	if ( trapezoid_ptrs_.at(1)->getVertexA() != fl::nan ) {
		trapezoid_ptrs_.at(1)->setVertexA(fl::nan);
		trapezoid_ptrs_.at(1)->setVertexA(fl::nan);
		trapezoid_ptrs_.at(1)->setVertexA(fl::nan);
		trapezoid_ptrs_.at(1)->setVertexA(fl::nan);
		trapezoid_ptrs_.at(1)->setHeight(0.0f);
	}

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
