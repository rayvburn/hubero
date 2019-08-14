/*
 * Processor.cpp
 *
 *  Created on: Aug 7, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/Processor.h>

// ------------------------------------------------------------------- //

namespace sfm {
namespace fuzz {

// flags to check the `status register`
#define PROCESSOR_EXTRA_TERM_NONE 			(0x01)
#define PROCESSOR_EXTRA_TERM_OUTWARDS 		(0x02)
#define PROCESSOR_EXTRA_TERM_CROSS_FRONT 	(0x04)
#define PROCESSOR_EXTRA_TERM_CROSS_BEHIND 	(0x08)

Processor::Processor(): term_extra_status_(PROCESSOR_EXTRA_TERM_NONE) {
	// TODO Auto-generated constructor stub
	init(); // TODO: move init() to ctor()
}

// ------------------------------------------------------------------- //

void Processor::init() {

	/* Initialize engine */
	engine_.setName("SocialBehaviors");
	engine_.setDescription("");

	/* Initialize first input variable */
	location_.setName("location");
	location_.setDescription("");
	location_.setEnabled(true);
	location_.setRange(-IGN_PI, +IGN_PI);
	location_.setLockValueInRange(false);

	// `location` regions
	location_.addTerm(new fl::Ramp("back", 				IGN_DTOR(-180.0), 	IGN_DTOR(-160.0)));
	location_.addTerm(new fl::Trapezoid("back_right", 	IGN_DTOR(-180.0), 	IGN_DTOR(-150.0), 	IGN_DTOR(-120.0), 	IGN_DTOR(-90.0)));
	location_.addTerm(new fl::Trapezoid("front_right", 	IGN_DTOR(-120.0) 	IGN_DTOR(-90.0), 	IGN_DTOR(-30.0), 	IGN_DTOR(0.0)));
	location_.addTerm(new fl::Triangle("front", 		IGN_DTOR(-20.0), 	IGN_DTOR(0.0), 		IGN_DTOR(+20.0)));
	location_.addTerm(new fl::Trapezoid("front_left", 	IGN_DTOR(0.0) 		IGN_DTOR(30.0), 	IGN_DTOR(90.0), 	IGN_DTOR(120.0)));
	location_.addTerm(new fl::Trapezoid("back_left", 	IGN_DTOR(90.0) 		IGN_DTOR(120.0), 	IGN_DTOR(150.0), 	IGN_DTOR(180.0)));
	location_.addTerm(new fl::Ramp("back", 				IGN_DTOR(+160.0), 	IGN_DTOR(+180.0)));
	engine_.addInputVariable(&location_);

	/* Initialize second input variable */
	direction_.setName("direction");
	direction_.setDescription("");
	direction_.setEnabled(true);
	direction_.setRange(-IGN_PI, +IGN_PI);
	direction_.setLockValueInRange(false);

	// `direction` regions must be configured dynamically,
	// make it default at the start
	direction_.addTerm(new fl::Trapezoid("outwards"));
	direction_.addTerm(new fl::Trapezoid("cross_front"));
	// NOTE: `cross_center` is not a valid region (not useful for reasoning)
	direction_.addTerm(new fl::Trapezoid("cross_behind"));
	direction_.addTerm(new fl::Trapezoid("equal"));
	direction_.addTerm(new fl::Trapezoid("opposite"));
	engine_.addInputVariable(&direction_);

    /* Initialize output variable */
    social_behavior_.setName("mSteer");
    social_behavior_.setDescription("");
    social_behavior_.setEnabled(true);
    social_behavior_.setRange(0.000, 1.000);
    social_behavior_.setLockValueInRange(false);
    social_behavior_.setAggregation(new fl::Maximum);
    social_behavior_.setDefuzzifier(new fl::Centroid(100));
    social_behavior_.setDefaultValue(fl::nan);
    social_behavior_.setLockPreviousValue(false);
//    social_behavior_.addTerm(new fl::Ramp("left", 1.000, 0.000));
//    social_behavior_.addTerm(new fl::Ramp("right", 0.000, 1.000));
    engine_.addOutputVariable(&social_behavior_);

    /* Initialize rule block */
    rule_block_.setName("mamdani");
    rule_block_.setDescription("");
    rule_block_.setEnabled(true);
    rule_block_.setConjunction(fl::null);
    rule_block_.setDisjunction(fl::null);
    rule_block_.setImplication(new fl::AlgebraicProduct);
    rule_block_.setActivation(new fl::General);
//    rule_block_.addRule(fl::Rule::parse("if obstacle is left then mSteer is right", &engine_));
//    rule_block_.addRule(fl::Rule::parse("if obstacle is right then mSteer is left", &engine_));
    engine_.addRuleBlock(&rule_block_);

}

// ------------------------------------------------------------------- //

void Processor::updateRegions() {

	// calculate threshold angle values, normalize
	ignition::math::Angle gamma_eq(d_alpha_beta_angle_ - 2 * alpha_dir_); 	gamma_eq.Normalize();
	ignition::math::Angle gamma_opp(gamma_eq.Radian() - IGN_PI); 			gamma_opp.Normalize();
	ignition::math::Angle gamma_cc(IGN_PI - alpha_dir_);					gamma_cc.Normalize();

	// compute relative location (`side`)
	char side = decodeRelativeLocation(gamma_eq, gamma_opp, gamma_cc);

	// trapezoid's specific points (vertices), see `fuzzylite` doc for details:
	// https://fuzzylite.github.io/fuzzylite/d0/d26/classfl_1_1_trapezoid.html
	double a, b, c, d;

	uint8_t status = PROCESSOR_EXTRA_TERM_NONE;

	std::string params;
	bool extra_term_needed;

	/* - - - - - Terms sensitive to side changes - - - - - */
	// `outwards`
	status |= configureTermLocationDependent("outwards", side, gamma_eq.Radian(), gamma_opp.Radian());
//	rearrangeTerms("outwards", status, term_extra_status_);

	// `cross_front`
	status |= configureTermLocationDependent("cross_front", side, gamma_cc.Radian(), gamma_eq.Radian());
//	rearrangeTerms("cross_front", status, term_extra_status_);

	// `cross_behind`
	status |= configureTermLocationDependent("cross_behind", side, gamma_opp.Radian(), gamma_cc.Radian());
//	rearrangeTerms("cross_behind", status, term_extra_status_);

	rearrangeTerms(status, term_extra_status_);

	/* - - - - - Terms insensitive to side changes - - - - - */
	// `equal`
	configureTermLocationIndependent("equal", gamma_eq.Radian());

	// `opposite`
	configureTermLocationIndependent("opposite", gamma_opp.Radian());

}

// ------------------------------------------------------------------- //

// pass by value
char Processor::decodeRelativeLocation(ignition::math::Angle eq, const ignition::math::Angle opp,
		ignition::math::Angle cc) const {

	// shift values to the left so the `gamma_opp` angle is equal to -PI
	ignition::math::Angle shift;
	shift.Radian(-IGN_PI - opp.Radian());

	// calculate shifted values
	eq += shift;	eq.Normalize();
	cc += shift;	cc.Normalize();

	// decode
	if ( cc.Radian() >= eq.Radian() ) {
		return ('l');
	} else {
		return ('r');
	}

}

// ------------------------------------------------------------------- //

uint8_t Processor::configureTermLocationDependent(std::string name, const char &side, const ignition::math::Angle &gamma_start,
		const ignition::math::Angle &gamma_end) {

	std::string params;
	bool extra_term_needed;
	uint8_t status = PROCESSOR_EXTRA_TERM_NONE;

	// based on `side` argument value, let's calculate trapezoid parameters
	// and decide whether an additional term is needed
	if ( side == 'r' ) {
		std::tie(params, extra_term_needed) = calculateTrapezoid(gamma_start.Radian(), gamma_end.Radian());
	} else {
		std::tie(params, extra_term_needed) = calculateTrapezoid(gamma_end.Radian(), gamma_start.Radian());
	}

	// update a term of the given `name`
	direction_.getTerm(name)->configure(params);

	// check whether an additional term is needed
	if ( extra_term_needed ) {

		if ( side == 'r' ) {
			std::tie(params, std::ignore) = calculateTrapezoid(gamma_start.Radian(), gamma_end.Radian(), true);
		} else {
			std::tie(params, std::ignore) = calculateTrapezoid(gamma_end.Radian(), gamma_start.Radian(), true);
		}

		// save status value based on the `name`
		switch (name) {
		case("outwards"):
				status |= PROCESSOR_EXTRA_TERM_OUTWARDS;
				break;
		case("cross_front"):
				status |= PROCESSOR_EXTRA_TERM_CROSS_FRONT;
				break;
		case("cross_behind"):
				status |= PROCESSOR_EXTRA_TERM_CROSS_BEHIND;
				break;
		}

		// name of the term to check
		name += "_extra";
		if ( !direction_.hasTerm(name) ) {
			direction_.addTerm(new fl::Trapezoid(name));
		}
		direction_.getTerm(name)->configure(params);

	}

	return (status);

}

// ------------------------------------------------------------------- //

void Processor::configureTermLocationIndependent(const std::string &name, const ignition::math::Angle &gamma_center) {

	std::string params;
	std::tie(params, std::ignore) = calculateTrapezoid(gamma_center.Radian());
	direction_.getTerm(name)->configure(params);

}

// ------------------------------------------------------------------- //

//std::tuple<double, double, double, double> Processor::calculateTrapezoid(const double &center) const {
std::string Processor::calculateTrapezoid(const double &center) const {

	static const double INTERVAL = IGN_DTOR(5.0);
	return (calculateTrapezoid(center - INTERVAL, center + INTERVAL));

}

// ------------------------------------------------------------------- //

//std::tuple<double, double, double, double> Processor::calculateTrapezoid(const double &start,
std::tuple<std::string, bool> Processor::calculateTrapezoid(const double &start,
		const double &end, bool comp_breakdown) const {

	// GAP shifts region bounding point towards the region's center
	static const double GAP = IGN_DTOR(5.0);
	// INTERSECTION is an interval used to artificially extend the region's range
	static const double INTERSECTION = IGN_DTOR(15.0);

	// allocate variables
	ignition::math::Angle gamma;
	std::string str;

	// check whether `start` angle is smaller than `end` angle
	if ( start <= end ) {

		// normal operation
		gamma.Radian(start + GAP - INTERSECTION);	gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(start + GAP);					gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(end - GAP);					gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(end - GAP + INTERSECTION);		gamma.Normalize();		str += gamma.Radian();
		return (std::make_tuple(str, false));

	} else if ( !comp_breakdown ) {

		// NOTE: Seems that `start` is bigger than `end` so a `breakdown` of ranges occurred.
		// This kind of situation must be handled differently - see `fuzzylite`'s
		// `Trapezoid` `membership` method:
		// https://fuzzylite.github.io/fuzzylite/d0/d26/classfl_1_1_trapezoid.html#a266a40979ac36f6012efce935302e983
		// 2 separate trapezoids must be created (first with c = d and a != b,
		// second with a = b and c != d)
		gamma.Radian(end + GAP - INTERSECTION);	gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(end + GAP);				gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(IGN_PI);					gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(IGN_PI);					gamma.Normalize();		str += gamma.Radian();
		// FIXME: intersection range must moved to the opposite side of the x-axis
		// when <end; pi> range is too narrow
		return (std::make_tuple(str, true));

	} else {

		// compute the second part of the trapezoid
		gamma.Radian(-IGN_PI);					gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(-IGN_PI);					gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(end + GAP - INTERSECTION);	gamma.Normalize();		str += gamma.Radian(); 	str += " ";
		gamma.Radian(end + GAP);				gamma.Normalize();		str += gamma.Radian();
		// FIXME: consider `intersection` movement from the close-to-pi range
		return (std::make_tuple(str, false));

	}

}

// ------------------------------------------------------------------- //

// old `extra` terms deleter
bool Processor::rearrangeTerms(const std::string &name, const uint8_t &status_curr, uint8_t &status_global) {

//	// check whether the current status is not equal to the global one;
//	// if the condition is TRUE then some action must be performed
//	if ( status_global != status_curr ) {
//		// check whether the global status is different to `NONE`
//		if ( status_global != PROCESSOR_EXTRA_TERM_NONE ) {
//
//			// SO: the `status_curr` indicates that:
//			// some term has already been added as an `EXTRA` one so:
//			//	o	(I)  the old extra term should be deleted because another one added (base term
//			//			 of the `EXTRA` one changed)
//			// 	o	(II) the old extra term should be deleted
//
//			// NOTE: in fact it is easy to predict that the index of the term
//			// which should be deleted is 5 (there may be at most 7 terms (quantity,
//			// not an index number) in the (I) case). The reason is that the new term
//			// is always `pushed_back` to the Terms vector of the input variable.
//			direction_.removeTerm(5);
//
//		} else {
//
//			// some term should be added
//			// DO NOTHING, it has already been added in the `configureTermLocationDependent` method
//
//		}
//
//		return (true);
//
//	}
//
//	return (false);



}

// ------------------------------------------------------------------- //

// old `extra` terms deleter
bool Processor::rearrangeTerms(const uint8_t &status_curr, uint8_t &status_global) {

	// check whether the current status is not equal to the global one;
	// if the condition is TRUE then some action must be performed
	if ( status_global != status_curr ) {

		// check the global status
		if ( status_curr | PROCESSOR_EXTRA_TERM_NONE ) {

			// all extra terms should be deleted
			if ( direction_.hasTerm("outwards_extra") ) {

			}
			if ( direction_.hasTerm("cross_front_extra") ) {

			}

		} else {



		}

//		if ( status_curr | PROCESSOR_EXTRA_TERM_OUTWARDS ) {
//
//			if ( status_curr | PROCESSOR_EXTRA_TERM_OUTWARDS ) {
//
//			}
//
//		} else {
//			// delete
//		}
//
//		if ( status_curr | PROCESSOR_EXTRA_TERM_CROSS_FRONT ) {
//
//
//
//		}
//
//		if ( status_curr | PROCESSOR_EXTRA_TERM_CROSS_BEHIND ) {
//
//
//
//		}


	}

}

// ------------------------------------------------------------------- //

Processor::~Processor() {}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
