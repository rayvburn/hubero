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
//#define PROCESSOR_EXTRA_TERM_NONE 			(0x01)
//#define PROCESSOR_EXTRA_TERM_OUTWARDS 		(0x02)
//#define PROCESSOR_EXTRA_TERM_CROSS_FRONT 	(0x04)
//#define PROCESSOR_EXTRA_TERM_CROSS_BEHIND 	(0x08)

Processor::Processor():
//		  term_extra_status_(PROCESSOR_EXTRA_TERM_NONE),
		  d_alpha_beta_angle_(0.0),
		  alpha_dir_(0.0)
{
	// FIXME: move init() to ctor()
	init();
}

// ------------------------------------------------------------------- //

void Processor::checkFl() {

	std::cout << "PROCESSOR_CHECK_FL" << std::endl;

	/* Initialize engine */
	engine_.setName("SocialBehaviors");
	engine_.setDescription("");

	/* Initialize second input variable */
	direction_.setName("direction");
	direction_.setDescription("");
	direction_.setEnabled(true);
	direction_.setRange(-IGN_PI, +IGN_PI);
	direction_.setLockValueInRange(false);
	engine_.addInputVariable(&direction_);

	/// TEST1
//	const double x = IGN_DTOR(-25.0);
//	direction_.addTerm(new fl::Trapezoid("test1", IGN_DTOR(-180.0), IGN_DTOR(-20.0), IGN_DTOR(+20.0), IGN_DTOR(+180.0)));
//	direction_.addTerm(new fl::Trapezoid("test2", IGN_DTOR(-180.0), IGN_DTOR(-100.0), IGN_DTOR(+100.0), IGN_DTOR(+180.0)));
//	std::cout << "highest membership for x=" << x << ": " << direction_.highestMembership(x)->getName() << std::endl;
//	std::cout << "memberships for x=" << x << " are: " << direction_.fuzzify(x) << std::endl;

	/// TEST2
	/* Below experiment shows that 2 terms can have the same name and the membership function
	 * will be calculated properly as long as terms ranges do not intersect. Output:
	PROCESSOR_CHECK_FL
	highest membership for x1=-2.18166: test
	memberships for x1=-2.18166 are: 0.625/test + 0.000/test
	highest membership for x2=2.18166: test
	memberships for x2=2.18166 are: 0.000/test + 0.500/test
	 */
//
//	const double x1 = IGN_DTOR(-125.0);
//	const double x2 = IGN_DTOR(+125.0);
//	direction_.addTerm(new fl::Trapezoid("test", IGN_DTOR(-180.0), IGN_DTOR(-160.0), IGN_DTOR(-140.0), IGN_DTOR(-100.0)));
//	direction_.addTerm(new fl::Ramp("test", 	 IGN_DTOR(100.0), IGN_DTOR(150.0)));
//	std::cout << "highest membership for x1=" << x1 << ": " << direction_.highestMembership(x1)->getName() << std::endl;
//	std::cout << "memberships for x1=" << x1 << " are: " << direction_.fuzzify(x1) << std::endl;
//	std::cout << std::endl;
//	std::cout << "highest membership for x2=" << x2 << ": " << direction_.highestMembership(x2)->getName() << std::endl;
//	std::cout << "memberships for x2=" << x2 << " are: " << direction_.fuzzify(x2) << std::endl;
//	std::cout << std::endl;
//	std::cout << std::endl;

	/// TEST3: term goes out of range when lock value in range is TRUE
	/*
	direction_.setLockValueInRange(true);
	const double x1 = IGN_DTOR(-125.0);
	const double x2 = IGN_DTOR(+181.0);
	direction_.addTerm(new fl::Trapezoid("test", IGN_DTOR(-180.0), IGN_DTOR(-160.0), IGN_DTOR(-140.0), IGN_DTOR(-100.0)));
	direction_.addTerm(new fl::Ramp("test", 	 IGN_DTOR(100.0), IGN_DTOR(180.0)));
	std::cout << "highest membership for x1=" << x1 << ": " << direction_.highestMembership(x1)->getName() << std::endl;
	std::cout << "memberships for x1=" << x1 << " are: " << direction_.fuzzify(x1) << std::endl;
	std::cout << std::endl;
	std::cout << "highest membership for x2=" << x2 << ": " << direction_.highestMembership(x2)->getName() << std::endl;
	std::cout << "memberships for x2=" << x2 << " are: " << direction_.fuzzify(x2) << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;

	//	Output:
//	highest membership for x1=-2.18166: test
//	memberships for x1=-2.18166 are: 0.625/test + 0.000/test
//	highest membership for x2=3.15905: test
//	memberships for x2=3.15905 are: 0.000/test + 1.000/test
	// SO THE ARGUMENT does exceed the allowable range but the membership function is calculated properly
	*/

	/// TEST 4: syntax of the uninitialized term
	/*
	fl::Trapezoid* trap = new fl::Trapezoid("test1");
	std::cout << "\n\nterm: " << trap->getName() << "'s configuration is: " << trap->parameters() << "\n\n";
	delete trap;
	// Output:
//	term: test1's configuration is: nan nan nan nan
	 */


	int i = 0;
	while (i++ < 9000000); // delay

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
//	location_.addTerm(new fl::Ramp("back", 				IGN_DTOR(-180.0), 	IGN_DTOR(-160.0)));
	location_.addTerm(new fl::Triangle("back",			IGN_DTOR(-180.0), 	IGN_DTOR(-180.0),  	IGN_DTOR(-160.0)));
	location_.addTerm(new fl::Trapezoid("back_right", 	IGN_DTOR(-180.0), 	IGN_DTOR(-150.0), 	IGN_DTOR(-120.0), 	IGN_DTOR(-90.0)));
	location_.addTerm(new fl::Trapezoid("front_right", 	IGN_DTOR(-120.0), 	IGN_DTOR(-90.0), 	IGN_DTOR(-30.0), 	IGN_DTOR(0.0)));
	location_.addTerm(new fl::Triangle("front", 		IGN_DTOR(-20.0), 	IGN_DTOR(0.0), 		IGN_DTOR(+20.0)));
	location_.addTerm(new fl::Trapezoid("front_left", 	IGN_DTOR(0.0), 		IGN_DTOR(30.0), 	IGN_DTOR(90.0), 	IGN_DTOR(120.0)));
	location_.addTerm(new fl::Trapezoid("back_left", 	IGN_DTOR(90.0),		IGN_DTOR(120.0), 	IGN_DTOR(150.0), 	IGN_DTOR(180.0)));
//	location_.addTerm(new fl::Ramp("back", 				IGN_DTOR(+160.0), 	IGN_DTOR(+180.0)));
	location_.addTerm(new fl::Triangle("back",			IGN_DTOR(+160.0), 	IGN_DTOR(+180.0),  	IGN_DTOR(+180.0)));
	engine_.addInputVariable(&location_);

	/* Initialize second input variable */
	direction_.setName("direction");
	direction_.setDescription("");
	direction_.setEnabled(true);
	direction_.setRange(-IGN_PI, +IGN_PI);
	direction_.setLockValueInRange(false);

	// `direction` regions must be configured dynamically,
	// make it default at the start
//	direction_.addTerm(new fl::Trapezoid("outwards"));
//	direction_.addTerm(new fl::Trapezoid("cross_front"));
//	// NOTE: `cross_center` is not a valid region (not useful for reasoning)
//	direction_.addTerm(new fl::Trapezoid("cross_behind"));
//	direction_.addTerm(new fl::Trapezoid("equal"));
//	direction_.addTerm(new fl::Trapezoid("opposite"));
	// ---
	// `outwards`
	direction_.addTerm(trapezoid_out_.getTrapezoids().at(0));
	direction_.addTerm(trapezoid_out_.getTrapezoids().at(1));
	// `cross_front`
	direction_.addTerm(trapezoid_cf_.getTrapezoids().at(0));
	direction_.addTerm(trapezoid_cf_.getTrapezoids().at(1));
	// `cross_behind`
	direction_.addTerm(trapezoid_cb_.getTrapezoids().at(0));
	direction_.addTerm(trapezoid_cb_.getTrapezoids().at(1));
	// `equal`
	direction_.addTerm(trapezoid_eq_.getTrapezoids().at(0));
	direction_.addTerm(trapezoid_eq_.getTrapezoids().at(1));
	// `opposite`
	direction_.addTerm(trapezoid_opp_.getTrapezoids().at(0));
	direction_.addTerm(trapezoid_opp_.getTrapezoids().at(1));
	// ---
	engine_.addInputVariable(&direction_);

    /* Initialize output variable */
    social_behavior_.setName("behavior");
    social_behavior_.setDescription("");
    social_behavior_.setEnabled(true);
    social_behavior_.setRange(0.000, 9.000); // FIXME: change if any term changes
    social_behavior_.setLockValueInRange(false);
    social_behavior_.setAggregation(new fl::Maximum);
    social_behavior_.setDefuzzifier(new fl::Centroid(100));
    social_behavior_.setDefaultValue(fl::nan);
    social_behavior_.setLockPreviousValue(false);

//    social_behavior_.addTerm(new fl::Ramp("left", 1.000, 0.000));
//    social_behavior_.addTerm(new fl::Ramp("right", 0.000, 1.000));

    //
    social_behavior_.addTerm(new fl::Trapezoid("turn_left", 				0.0, 0.0, 1.0, 1.1));
    social_behavior_.addTerm(new fl::Trapezoid("turn_left_accelerate", 		0.9, 1.0, 2.0, 2.1));
    social_behavior_.addTerm(new fl::Trapezoid("accelerate", 				1.9, 2.0, 3.0, 3.1));
    social_behavior_.addTerm(new fl::Trapezoid("go_along", 					2.9, 3.0, 4.0, 4.1));
    social_behavior_.addTerm(new fl::Trapezoid("decelerate", 				3.9, 4.0, 5.0, 5.1));
    social_behavior_.addTerm(new fl::Trapezoid("stop", 						4.9, 5.0, 6.0, 6.1));
    social_behavior_.addTerm(new fl::Trapezoid("turn_right_deccelerate",	5.9, 6.0, 7.0, 7.1));
    social_behavior_.addTerm(new fl::Trapezoid("turn_right", 				6.9, 7.0, 8.0, 8.1));
    social_behavior_.addTerm(new fl::Trapezoid("turn_right_accelerate", 	7.9, 8.0, 9.0, 9.0));

    engine_.addOutputVariable(&social_behavior_);

    // output regions
    //
    /*
     X turn_left
     X go_along
     X accelerate
     X turn_left_accelerate
     X turn_right_accelerate
     X turn_right
     X turn_right_deccelerate
     X stop
     X decelerate
     */

    /* Initialize rule block */
    rule_block_.setName("mamdani");
    rule_block_.setDescription("");
    rule_block_.setEnabled(true);
    rule_block_.setConjunction(new fl::Minimum); 			// fuzzylite/fuzzylite/fl/norm/t
    rule_block_.setDisjunction(new fl::AlgebraicSum);		// fuzzylite/fuzzylite/fl/norm/s
    rule_block_.setImplication(new fl::AlgebraicProduct);
    rule_block_.setActivation(new fl::General);				// https://fuzzylite.github.io/fuzzylite/df/d4b/classfl_1_1_activation.html
    /*
      	terminate called after throwing an instance of 'fl::Exception'
			what():  [conjunction error] the following rule requires a conjunction operator:
		location is front_right and direction is cross_front
		{at /src/rule/Antecedent.cpp::activationDegree() [line:114]}
     */
    //
    // template
    // rule_block_.addRule(fl::Rule::parse("if location is X and direction is Y then behavior is Z", &engine_));
    //
    // location - `front_right`
    rule_block_.addRule(fl::Rule::parse("if location is front_right and direction is cross_front then behavior is turn_right_deccelerate", &engine_)); 	// 1
    rule_block_.addRule(fl::Rule::parse("if location is front_right and direction is cross_behind then behavior is turn_left", &engine_)); 				// 2
    rule_block_.addRule(fl::Rule::parse("if location is front_right and direction is equal then behavior is go_along", &engine_)); 						// 3
    rule_block_.addRule(fl::Rule::parse("if location is front_right and direction is opposite then behavior is turn_left", &engine_)); 					// 4
    rule_block_.addRule(fl::Rule::parse("if location is front_right and direction is outwards then behavior is turn_left", &engine_)); 					// 9
    // location - `back_right`
    rule_block_.addRule(fl::Rule::parse("if location is back_right and direction is cross_front then behavior is accelerate", &engine_)); 				// 5
    rule_block_.addRule(fl::Rule::parse("if location is back_right and direction is cross_behind then behavior is turn_left_accelerate", &engine_)); 	// 6
    rule_block_.addRule(fl::Rule::parse("if location is back_right and direction is equal then behavior is go_along", &engine_)); 						// 7
    rule_block_.addRule(fl::Rule::parse("if location is back_right and direction is opposite then behavior is go_along", &engine_)); 					// 8
    rule_block_.addRule(fl::Rule::parse("if location is back_right and direction is outwards then behavior is go_along", &engine_)); 					// 10
    // location - `front_left`
    rule_block_.addRule(fl::Rule::parse("if location is front_left and direction is cross_behind then behavior is turn_right_accelerate", &engine_)); 	// 11
    rule_block_.addRule(fl::Rule::parse("if location is front_left and direction is equal then behavior is go_along", &engine_));						// 12
    rule_block_.addRule(fl::Rule::parse("if location is front_left and direction is opposite then behavior is turn_right", &engine_));					// 13
    rule_block_.addRule(fl::Rule::parse("if location is front_left and direction is outwards then behavior is turn_right_deccelerate", &engine_));		// 14
    // location - `back_left`
    rule_block_.addRule(fl::Rule::parse("if location is back_left and direction is equal then behavior is go_along", &engine_));						// 15
    rule_block_.addRule(fl::Rule::parse("if location is back_left and direction is opposite then behavior is go_along", &engine_));						// 16
    // location - `front`
    rule_block_.addRule(fl::Rule::parse("if location is front and direction is outwards then behavior is stop", &engine_));								// 17
    rule_block_.addRule(fl::Rule::parse("if location is front and direction is cross_front then behavior is stop", &engine_));							// 17b
    rule_block_.addRule(fl::Rule::parse("if location is front and direction is equal then behavior is decelerate", &engine_));							// 18
    rule_block_.addRule(fl::Rule::parse("if location is front and direction is opposite then behavior is turn_right", &engine_));						// 19
    // location - `back`
    rule_block_.addRule(fl::Rule::parse("if location is back and direction is outwards then behavior is go_along", &engine_));							// 20
    rule_block_.addRule(fl::Rule::parse("if location is back and direction is cross_behind then behavior is go_along", &engine_));						// 20b
    // apply rules
    std::cout << "\nfuzzylite's RULE BLOCK: " << rule_block_.toString() << std::endl << std::endl;
    engine_.addRuleBlock(&rule_block_);
//    engine_.configure("AlgebraicProduct", "AlgebraicSum", "AlgebraicProduct", "AlgebraicSum", "Centroid");
//    engine_.configure("conj", "disj", "implic", "aggreg", "defuzz", "activ");
    // FIXME
    engine_.configure("AlgebraicProduct", "AlgebraicSum", "AlgebraicProduct", "Maximum", "Centroid", "General");
    std::cout << "inputs: " << engine_.numberOfInputVariables() << "\toutputs: " << engine_.numberOfOutputVariables() << "\trule_blocks: " << engine_.numberOfRuleBlocks() << std::endl << std::endl;

}

// ------------------------------------------------------------------- //

void Processor::setDirectionBeta(const double &dir_beta) {
	beta_dir_ = dir_beta;
}

// ------------------------------------------------------------------- //

void Processor::setDirectionAlpha(const double &dir_alpha) {
	alpha_dir_ = dir_alpha;
}

// ------------------------------------------------------------------- //

void Processor::setRelativeLocation(const double &beta_location_relative_to_alpha_dir_angle) {
	d_alpha_beta_angle_ = beta_location_relative_to_alpha_dir_angle;
}

// ------------------------------------------------------------------- //

void Processor::updateRegions() {

	// calculate threshold angle values, normalize
	ignition::math::Angle gamma_eq(d_alpha_beta_angle_ - 2 * alpha_dir_); 	gamma_eq.Normalize();
	ignition::math::Angle gamma_opp(gamma_eq.Radian() - IGN_PI); 			gamma_opp.Normalize();
	ignition::math::Angle gamma_cc(IGN_PI - alpha_dir_);					gamma_cc.Normalize();

	// compute relative location (`side`)
	char side = decodeRelativeLocation(gamma_eq, gamma_opp, gamma_cc);

//	// trapezoid's specific points (vertices), see `fuzzylite` doc for details:
//	// https://fuzzylite.github.io/fuzzylite/d0/d26/classfl_1_1_trapezoid.html
//	double a, b, c, d;
//
//	uint8_t status = PROCESSOR_EXTRA_TERM_NONE;
//
//	std::string params;
//	bool extra_term_needed;

	/* - - - - - Terms sensitive to side changes - - - - - */
	// `outwards`
//	status |= configureTermLocationDependent("outwards", side, gamma_eq.Radian(), gamma_opp.Radian());
	//rearrangeTerms("outwards", status, term_extra_status_);

	// `cross_front`
//	status |= configureTermLocationDependent("cross_front", side, gamma_cc.Radian(), gamma_eq.Radian());
	//rearrangeTerms("cross_front", status, term_extra_status_);

	// `cross_behind`
//	status |= configureTermLocationDependent("cross_behind", side, gamma_opp.Radian(), gamma_cc.Radian());
	//rearrangeTerms("cross_behind", status, term_extra_status_);


//	rearrangeTerms(status, term_extra_status_);


	/* - - - - - Terms insensitive to side changes - - - - - */
	// `equal`
//	trapezoid_out_.upda
//	configureTermLocationIndependent("equal", gamma_eq.Radian());

	// `opposite`
//	configureTermLocationIndependent("opposite", gamma_opp.Radian());

	/* - - - - - Terms sensitive to side changes - - - - - */
	// `outwards`
	trapezoid_out_.update(side, gamma_eq, gamma_opp);
	// `cross_front`
	trapezoid_cf_.update(side, gamma_cc, gamma_eq);
	// `cross_behind`
	trapezoid_cb_.update(side, gamma_opp, gamma_cc);

	/* - - - - - Terms insensitive to side changes - - - - - */
	// `equal`
	trapezoid_eq_.update(gamma_eq);
	// `opposite`
	trapezoid_opp_.update(gamma_opp);

	// - - - - - - print meaningful data
	// calculate the gamma angle for the current alpha-beta configuration
	// FIXME: debugging only
//	ignition::math::Angle gamma(d_alpha_beta_angle_ - alpha_dir_ - beta_dir_);	gamma.Normalize();
//	std::cout << "gamma_eq: " << gamma_eq.Radian() << "\t\tgamma_opp: " << gamma_opp.Radian() << "\t\tgamma_cc: " << gamma_cc.Radian() << std::endl;
//	if ( side == 'l' ) {
//
//		if ( 		gamma.Radian() < gamma_eq.Radian()  && gamma.Radian() > gamma_opp.Radian() ) {
//			std::cout << "LEFT - OUTWARDS" << std::endl;
//		} else if ( gamma.Radian() < gamma_opp.Radian() && gamma.Radian() > gamma_cc.Radian() ) {
//			std::cout << "LEFT - CROSS_BEHIND" << std::endl;
//		} else if ( gamma.Radian() < gamma_cc.Radian() && gamma.Radian() > gamma_eq.Radian() ) {
//			std::cout << "LEFT - CROSS_FRONT" << std::endl;
//		}
//
//	} else if ( side == 'r' ) {
//
//		if ( 		gamma.Radian() > gamma_eq.Radian()  && gamma.Radian() < gamma_opp.Radian() ) {
//			std::cout << "RIGHT - OUTWARDS" << std::endl;
//		} else if ( gamma.Radian() > gamma_opp.Radian() && gamma.Radian() < gamma_cc.Radian() ) {
//			std::cout << "RIGHT - CROSS_BEHIND" << std::endl;
//		} else if ( gamma.Radian() > gamma_cc.Radian()  && gamma.Radian() < gamma_eq.Radian() ) {
//			std::cout << "RIGHT - CROSS_FRONT" << std::endl;
//		}
//
//	}
	// - - - - - -

}

// ------------------------------------------------------------------- //

void Processor::process() {

	// update the location input variable
	location_.setValue(fl::scalar(d_alpha_beta_angle_));

	// update `direction_` regions according to value previously set
	updateRegions();

	// calculate the gamma angle for the current alpha-beta configuration
	ignition::math::Angle gamma(d_alpha_beta_angle_ - alpha_dir_ - beta_dir_);	gamma.Normalize();
    direction_.setValue(fl::scalar(gamma.Radian()));

    // execute fuzzy calculations
    engine_.process();

    // - - - - - - print meaningful data
    // FIXME: debugging only
//    std::cout << "location\t value: " << location_.getValue() << "\tmemberships: " << location_.fuzzify(location_.getValue()) << std::endl;
//    std::cout << "direction\t value: " << direction_.getValue() << "\tmemberships: " << direction_.fuzzify(direction_.getValue()) << std::endl;
//    std::cout << "output\t\t value: " << social_behavior_.getValue();
//	std::string beh;
//    double output = static_cast<double>(social_behavior_.getValue());
//	if ( output  < 1.0 ) {
//		beh = "turn_left";
//	} else if ( output < 2.0 ) {
//		beh = "turn_left_accelerate";
//	} else if ( output < 3.0 ) {
//		beh = "accelerate";
//	} else if ( output < 4.0 ) {
//		beh = "go_along";
//	} else if ( output < 5.0 ) {
//		beh = "decelerate";
//	} else if ( output < 6.0 ) {
//		beh = "stop";
//	} else if ( output < 7.0 ) {
//		beh = "turn_right_deccelerate";
//	} else if ( output < 8.0 ) {
//		beh = "turn_right";
//	} else if ( output < 9.0 ) {
//		beh = "turn_right_accelerate";
//	}
//	std::cout << "\t" << beh << std::endl;
    // - - - - - -

}

// ------------------------------------------------------------------- //

std::vector<std::tuple<std::string, double> > Processor::getOutput() const {

	std::vector<std::tuple<std::string, double> > results;

	/*
	social_behavior_.addTerm(new fl::Trapezoid(, 				0.0, 0.0, 1.0, 1.1));
    social_behavior_.addTerm(new fl::Trapezoid(, 		0.9, 1.0, 2.0, 2.1));
    social_behavior_.addTerm(new fl::Trapezoid(, 				1.9, 2.0, 3.0, 3.1));
    social_behavior_.addTerm(new fl::Trapezoid(, 					2.9, 3.0, 4.0, 4.1));
    social_behavior_.addTerm(new fl::Trapezoid(, 				3.9, 4.0, 5.0, 5.1));
    social_behavior_.addTerm(new fl::Trapezoid(, 						4.9, 5.0, 6.0, 6.1));
    social_behavior_.addTerm(new fl::Trapezoid(,	5.9, 6.0, 7.0, 7.1));
    social_behavior_.addTerm(new fl::Trapezoid(, 				6.9, 7.0, 8.0, 8.1));
    social_behavior_.addTerm(new fl::Trapezoid(, 	7.9, 8.0, 9.0, 9.0));
	 */

//	int units = social_behavior_.getValue() / (std::fabs(social_behavior_.getValue()));
	// TODO: consider possibility of double membership

	std::string beh;

	double output = static_cast<double>(social_behavior_.getValue());
	if ( output  < 1.0 ) {
		beh = "turn_left";
	} else if ( output < 2.0 ) {
		beh = "turn_left_accelerate";
	} else if ( output < 3.0 ) {
		beh = "accelerate";
	} else if ( output < 4.0 ) {
		beh = "go_along";
	} else if ( output < 5.0 ) {
		beh = "decelerate";
	} else if ( output < 6.0 ) {
		beh = "stop";
	} else if ( output < 7.0 ) {
		beh = "turn_right_deccelerate";
	} else if ( output < 8.0 ) {
		beh = "turn_right";
	} else if ( output < 9.0 ) {
		beh = "turn_right_accelerate";
	}
	std::tuple<std::string, double> tup;
	std::get<0>(tup) = beh;
	std::get<1>(tup) = 0;
	results.push_back(tup);

	return (results);

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

//uint8_t Processor::configureTermLocationDependent(std::string name, const char &side, const ignition::math::Angle &gamma_start,
//		const ignition::math::Angle &gamma_end) {
//
//
//
//}

// ------------------------------------------------------------------- //

//void Processor::configureTermLocationIndependent(const std::string &name, const ignition::math::Angle &gamma_center) {
//
//
//
//}

// ------------------------------------------------------------------- //

//std::tuple<double, double, double, double> Processor::calculateTrapezoid(const double &center) const {
//std::string Processor::calculateTrapezoid(const double &center) const {
//
//
//
//}

// ------------------------------------------------------------------- //

//std::tuple<double, double, double, double> Processor::calculateTrapezoid(const double &start,
//std::tuple<std::string, bool> Processor::calculateTrapezoid(const double &start,
//		const double &end, bool comp_breakdown) const {
//
//
//}

// ------------------------------------------------------------------- //

/*
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

	return (false);

}
*/


// ------------------------------------------------------------------- //

/*
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

	return (false);

}
*/


// ------------------------------------------------------------------- //

Processor::~Processor() {}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
