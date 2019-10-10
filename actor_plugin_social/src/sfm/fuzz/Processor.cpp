/*
 * Processor.cpp
 *
 *  Created on: Aug 7, 2019
 *      Author: rayvburn
 */

#include <sfm/fuzz/Processor.h>
#include <sfm/fuzz/Regions.h>

// ------------------------------------------------------------------- //

namespace sfm {
namespace fuzz {

// flags to check the `status register`
//#define PROCESSOR_EXTRA_TERM_NONE 			(0x01)
//#define PROCESSOR_EXTRA_TERM_OUTWARDS 		(0x02)
//#define PROCESSOR_EXTRA_TERM_CROSS_FRONT 	(0x04)
//#define PROCESSOR_EXTRA_TERM_CROSS_BEHIND 	(0x08)

// #define PROCESSOR_PRINT_DEBUG_INFO

Processor::Processor(): alpha_dir_(0.0) {
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
	location_.addTerm(new fl::Triangle("back",			IGN_DTOR(-180.0), 	IGN_DTOR(-180.0),  	IGN_DTOR(-160.0)));
	location_.addTerm(new fl::Trapezoid("back_right", 	IGN_DTOR(-180.0), 	IGN_DTOR(-150.0), 	IGN_DTOR(-120.0), 	IGN_DTOR(-90.0)));
	location_.addTerm(new fl::Trapezoid("front_right", 	IGN_DTOR(-120.0), 	IGN_DTOR(-90.0), 	IGN_DTOR(-30.0), 	IGN_DTOR(0.0)));
	location_.addTerm(new fl::Triangle("front", 		IGN_DTOR(-20.0), 	IGN_DTOR(0.0), 		IGN_DTOR(+20.0)));
	location_.addTerm(new fl::Trapezoid("front_left", 	IGN_DTOR(0.0), 		IGN_DTOR(30.0), 	IGN_DTOR(90.0), 	IGN_DTOR(120.0)));
	location_.addTerm(new fl::Trapezoid("back_left", 	IGN_DTOR(90.0),		IGN_DTOR(120.0), 	IGN_DTOR(150.0), 	IGN_DTOR(180.0)));
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
	// TODO: VFH = vector field histogram
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
    social_behavior_.setDefuzzifier(new fl::Centroid()); // prev resolution was 100
    social_behavior_.setDefaultValue(fl::nan);
    social_behavior_.setLockPreviousValue(false);

    std::cout << "\t**********************************************\n";
    std::cout << "\t\tSocialBehavior configuration\n";
    // TODO: wrap into function
    // threshold values related to regions
    const double INTERSECTION = 0.1;
    double upper = static_cast<double>(FUZZ_BEH_TURN_LEFT);
    double lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("turn_left", 				lower, 					lower, 	upper, 	upper + INTERSECTION));
    std::cout << "\tturn_left: " << lower << " / " << upper << std::endl;

    upper = static_cast<double>(FUZZ_BEH_TURN_LEFT_ACCELERATE); lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("turn_left_accelerate", 		lower - INTERSECTION,	lower, 	upper, 	upper + INTERSECTION));
    std::cout << "\tturn_left_accelerate: " << lower << " / " << upper << std::endl;

    upper = static_cast<double>(FUZZ_BEH_ACCELERATE); lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("accelerate", 				lower - INTERSECTION,	lower, 	upper, 	upper + INTERSECTION));
    std::cout << "\taccelerate: " << lower << " / " << upper << std::endl;

    upper = static_cast<double>(FUZZ_BEH_GO_ALONG); lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("go_along", 					lower - INTERSECTION,	lower, 	upper, 	upper + INTERSECTION));
    std::cout << "\tgo_along: " << lower << " / " << upper << std::endl;

    upper = static_cast<double>(FUZZ_BEH_DECELERATE); lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("decelerate", 				lower - INTERSECTION,	lower, 	upper, 	upper + INTERSECTION));
    std::cout << "\tdecelerate: " << lower << " / " << upper << std::endl;

    upper = static_cast<double>(FUZZ_BEH_STOP); lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("stop", 						lower - INTERSECTION,	lower, 	upper, 	upper + INTERSECTION));
    std::cout << "\tstop: " << lower << " / " << upper << std::endl;

    upper = static_cast<double>(FUZZ_BEH_TURN_RIGHT_DECELERATE); lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("turn_right_decelerate",		lower - INTERSECTION,	lower, 	upper, 	upper + INTERSECTION));
    std::cout << "\tturn_right_decelerate: " << lower << " / " << upper << std::endl;

    upper = static_cast<double>(FUZZ_BEH_TURN_RIGHT); lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("turn_right", 				lower - INTERSECTION,	lower, 	upper, 	upper + INTERSECTION));
    std::cout << "\tturn_right: " << lower << " / " << upper << std::endl;

    upper = static_cast<double>(FUZZ_BEH_TURN_RIGHT_ACCELERATE); lower = upper - 1.0;
    social_behavior_.addTerm(new fl::Trapezoid("turn_right_accelerate", 	lower - INTERSECTION,	lower, 	upper, 	upper));
    std::cout << "\tturn_right_accelerate: " << lower << " / " << upper << std::endl;
    std::cout << "\t**********************************************\n";

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
     X turn_right_decelerate
     X stop
     X decelerate
     */

    /* Initialize rule block */
    rule_block_.setName("mamdani");
    rule_block_.setDescription("");
    rule_block_.setEnabled(true);
    rule_block_.setConjunction(new fl::Minimum); 			// fuzzylite/fuzzylite/fl/norm/t
    rule_block_.setDisjunction(new fl::AlgebraicSum);		// fuzzylite/fuzzylite/fl/norm/s
    // FIXME: AlgebraicProduct -> seems to choose not the highest membership term? IS THIS THE CAUSE?
    rule_block_.setImplication(new fl::Minimum); 			// AlgebraicProduct);
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
    /*
     * NOTE: It seems that there is a crucial aspect in `fuzzylite` Term naming convention.
     * The library immediately throws segfault after AddRule call was fed with a term name
     * consisting of a number (at least at the end, didn't check other cases).
     * This may be the problem with fl::Term class or Rule parser function.
     * The program crashes also when a Rule contains a name of a Term which has not
     * been defined yet (whose name is unknown).
     */
    std::cout << "\nfuzzylite's RULE BLOCK1: " << rule_block_.toString() << std::endl << std::endl;
    // location - `front_right`
    //																																	 previously: stop - doesnt work well
    //																																	 `turn_right_decelerate` - causes sticking and going in the same direction
    //																																	 `turn_right_accelerate`
    rule_block_.addRule(fl::Rule::parse("if location is front_right and (direction is cross_frontA 	or direction is cross_frontB ) 	then behavior is turn_right", 	&engine_)); // 1
    rule_block_.addRule(fl::Rule::parse("if location is front_right and (direction is cross_behindA or direction is cross_behindB) 	then behavior is turn_left", 				&engine_)); // 2
    rule_block_.addRule(fl::Rule::parse("if location is front_right and (direction is equalA 		or direction is equalB		 ) 	then behavior is go_along", 				&engine_)); // 3
    rule_block_.addRule(fl::Rule::parse("if location is front_right and (direction is oppositeA 	or direction is oppositeB	 )	then behavior is turn_left", 				&engine_)); // 4
    rule_block_.addRule(fl::Rule::parse("if location is front_right and (direction is outwardsA 	or direction is outwardsB	 )	then behavior is turn_left", 				&engine_)); // 9
    // location - `back_right`
    rule_block_.addRule(fl::Rule::parse("if location is back_right  and (direction is cross_frontA 	or direction is cross_frontB ) 	then behavior is accelerate", 				&engine_)); // 5
    rule_block_.addRule(fl::Rule::parse("if location is back_right  and (direction is cross_behindA or direction is cross_behindB) 	then behavior is turn_left_accelerate", 	&engine_)); // 6
    rule_block_.addRule(fl::Rule::parse("if location is back_right  and (direction is equalA 		or direction is equalB		 ) 	then behavior is go_along", 				&engine_)); // 7
    rule_block_.addRule(fl::Rule::parse("if location is back_right  and (direction is oppositeA 	or direction is oppositeB	 ) 	then behavior is go_along", 				&engine_)); // 8
    rule_block_.addRule(fl::Rule::parse("if location is back_right  and (direction is outwardsA 	or direction is outwardsB	 ) 	then behavior is go_along", 				&engine_)); // 10
    // location - `front_left`																														 turn_left
    //																																				 turn_right_decelerate
    rule_block_.addRule(fl::Rule::parse("if location is front_left  and (direction is cross_frontA 	or direction is cross_frontB ) 	then behavior is turn_right_accelerate", 	&engine_)); // EXTRA (added after few experiments although it may just strengthen another case)
    //																																				 // try acc
    rule_block_.addRule(fl::Rule::parse("if location is front_left  and (direction is cross_behindA or direction is cross_behindB) 	then behavior is go_along", 	&engine_)); // 11 // V1 (after front changes): turn_right_accelerate
    rule_block_.addRule(fl::Rule::parse("if location is front_left  and (direction is equalA 		or direction is equalB		 ) 	then behavior is go_along", 				&engine_)); // 12
    rule_block_.addRule(fl::Rule::parse("if location is front_left  and (direction is oppositeA 	or direction is oppositeB	 ) 	then behavior is turn_right", 				&engine_)); // 13
    rule_block_.addRule(fl::Rule::parse("if location is front_left  and (direction is outwardsA 	or direction is outwardsB	 ) 	then behavior is turn_right_decelerate", 	&engine_)); // 14
    // location - `back_left`
    rule_block_.addRule(fl::Rule::parse("if location is back_left   and (direction is equalA 		or direction is equalB		 ) 	then behavior is go_along", 				&engine_)); // 15
    rule_block_.addRule(fl::Rule::parse("if location is back_left   and (direction is oppositeA 	or direction is oppositeB	 ) 	then behavior is go_along", 				&engine_)); // 16
    // location - `front`
    rule_block_.addRule(fl::Rule::parse("if location is front 	    and (direction is outwardsA 	or direction is outwardsB	 ) 	then behavior is accelerate", 					&engine_)); // 17	// V1: stop
    //																																				 V2: 'turn_right_decelerate'
    rule_block_.addRule(fl::Rule::parse("if location is front 	    and (direction is cross_frontA 	or direction is cross_frontB ) 	then behavior is turn_right", 					&engine_)); // 17b	// V1: stop
    rule_block_.addRule(fl::Rule::parse("if location is front 	    and (direction is equalA 		or direction is equalB		 ) 	then behavior is decelerate", 				&engine_)); // 18
    rule_block_.addRule(fl::Rule::parse("if location is front 	    and (direction is oppositeA 	or direction is oppositeB	 ) 	then behavior is turn_right", 				&engine_)); // 19
    // location - `back`
    rule_block_.addRule(fl::Rule::parse("if location is back 	    and (direction is outwardsA 	or direction is outwardsB	 ) 	then behavior is go_along", 				&engine_)); // 20
    rule_block_.addRule(fl::Rule::parse("if location is back 	    and (direction is cross_behindA or direction is cross_behindB) 	then behavior is go_along", 				&engine_)); // 20b
    // apply rules
    std::cout << "\nfuzzylite's RULE BLOCK: " << rule_block_.toString() << std::endl << std::endl;
    engine_.addRuleBlock(&rule_block_);
//    engine_.configure("AlgebraicProduct", "AlgebraicSum", "AlgebraicProduct", "AlgebraicSum", "Centroid");
//    engine_.configure("conj", "disj", "implic", "aggreg", "defuzz", "activ");
    // FIXME: below does not overwrite rule block's settings
    engine_.configure("AlgebraicProduct", "AlgebraicSum", "AlgebraicProduct", "Maximum", "Centroid", "General");
    std::cout << "inputs: " << engine_.numberOfInputVariables() << "\toutputs: " << engine_.numberOfOutputVariables() << "\trule_blocks: " << engine_.numberOfRuleBlocks() << std::endl << std::endl;

}

// ------------------------------------------------------------------- //

bool Processor::load(const double &dir_alpha, const std::vector<double> &dir_beta_v,
					 const std::vector<double> &rel_loc_v, const std::vector<double> &dist_angle_v)
{

	// clear output vector
	output_v_.clear();

	// the same length is a MUST
	if ( (dir_beta_v.size() == rel_loc_v.size()) && (rel_loc_v.size() == dist_angle_v.size()) ) {
		alpha_dir_ = dir_alpha;
		beta_dir_ = dir_beta_v;
		rel_loc_ = rel_loc_v;
		d_alpha_beta_angle_ = dist_angle_v;
		return (true);
	}

	// reset just in case
	alpha_dir_ = 0.0;
	beta_dir_.clear();
	rel_loc_.clear();
	d_alpha_beta_angle_.clear();

	return (false);

}

// ------------------------------------------------------------------- //

void Processor::process() {

	// iterate over all vector elements (all vectors have the same size);
	// beta_dir's size is an arbitrarily chosen count reference here
	for ( size_t i = 0; i < beta_dir_.size(); i++ ) {

		// reset the output region name
		std::string term_name = "";

		// update the location input variable
		location_.setValue(fl::scalar(rel_loc_.at(i)));

		// update `direction_` regions according to value previously set
		updateRegions(alpha_dir_, beta_dir_.at(i), d_alpha_beta_angle_.at(i), rel_loc_.at(i));

		// calculate the gamma angle for the current alpha-beta configuration
		ignition::math::Angle gamma(d_alpha_beta_angle_.at(i) - alpha_dir_ - beta_dir_.at(i));
		gamma.Normalize();
		direction_.setValue(fl::scalar(gamma.Radian()));

		// execute fuzzy calculations
		engine_.process();

		// - - - - - - print meaningful data

		#ifdef PROCESSOR_PRINT_DEBUG_INFO
		// FIXME: debugging only
		std::cout << "location\t value: " << location_.getValue() << "\tmemberships: " << location_.fuzzify(location_.getValue()) << std::endl;
		std::cout << "direction\t value: " << direction_.getValue() << "\tmemberships: " << direction_.fuzzify(direction_.getValue()) << std::endl;
		std::cout << "output\t\t value: " << social_behavior_.getValue() << std::endl;
		std::cout << "\t\tfuzzyOut: " << social_behavior_.fuzzyOutputValue();
		#endif

		fl::scalar y_highest_temp = fl::nan;
		fl::Term* term_highest_ptr = social_behavior_.highestMembership(social_behavior_.getValue(), &y_highest_temp);

		// check whether proper term was found, if not - `nullptr` will be detected
		if ( term_highest_ptr != nullptr ) {
			term_name = term_highest_ptr->getName();
		}

		#ifdef PROCESSOR_PRINT_DEBUG_INFO
		std::cout << "\n\t\tname: " << output_term_name_ << "\theight: " << y_highest_temp << std::endl;
		#endif

		// NOTE: fl::variable::fuzzyOutputValue() returns a list of available terms
		// with a corresponding membership
		// WHEREAS fl::variable fl::variable::fuzzify(fl::scalar) returns the
		// same list but with NORMALIZED membership?
		// the effect is as: 	fuzzyOutputValue()	-> 	0.222/turn_right_decelerate
		//						fuzzify(fl::scalar)	-> 	1.000/turn_right_decelerate
		// when only single term has non-zero membership.
		// When multiple (2) terms have non-zero membership then results
		// are as follow:
		// fuzzyOutputValue()	-> 	0.239/turn_left_accelerate + 0.266/accelerate + 0.541/go_along
		// fuzzify(fl::scalar)	-> 	0.000/turn_left_accelerate + 0.000/accelerate + 1.000/go_along

		// FIXME: make it like the absolute function (decreasing on both side of the edge - 0.5)
		double fitness = static_cast<double>(social_behavior_.getValue());
		fitness = fitness - std::floor(fitness);

		output_v_.push_back(std::make_tuple(term_name, fitness));

	}

}

// ------------------------------------------------------------------- //

std::vector<std::tuple<std::string, double> > Processor::getOutput() const {
	return (output_v_);
}

// ------------------------------------------------------------------- //

Processor::~Processor() {}

// ------------------------------------------------------------------- //

void Processor::updateRegions(const double &alpha_dir, const double &beta_dir, const double &d_alpha_beta_angle, const double &rel_loc) {

	// calculate threshold angle values, normalize
	ignition::math::Angle gamma_eq(d_alpha_beta_angle - 2 * alpha_dir); 	gamma_eq.Normalize();
	ignition::math::Angle gamma_opp(gamma_eq.Radian() - IGN_PI); 			gamma_opp.Normalize();
	ignition::math::Angle gamma_cc(IGN_PI - alpha_dir);					gamma_cc.Normalize();

	// compute relative location (`side`)
	char side = decodeRelativeLocation(rel_loc); // decodeRelativeLocation(gamma_eq, gamma_opp, gamma_cc);

//	// trapezoid's specific points (vertices), see `fuzzylite` doc for details:
//	// https://fuzzylite.github.io/fuzzylite/d0/d26/classfl_1_1_trapezoid.html

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
	ignition::math::Angle gamma(d_alpha_beta_angle - alpha_dir - beta_dir);	gamma.Normalize();

#ifdef PROCESSOR_PRINT_DEBUG_INFO
	std::cout << "gamma_eq: " << gamma_eq.Radian() << "\t\tgamma_opp: " << gamma_opp.Radian() << "\t\tgamma_cc: " << gamma_cc.Radian() << std::endl;
	// print only `side`
	if ( side == 'l' ) {
		std::cout << "----LEFT" << std::endl;
	} else if ( side == 'r' ) {
		std::cout << "----RIGHT" << std::endl;
	}
#endif
	// - - - - - -

}

// ------------------------------------------------------------------- //

char Processor::decodeRelativeLocation(const double &rel_loc) const {

	if ( rel_loc <= 0.0 ) {
		return('r'); 	// right side
	} else if ( rel_loc > 0.0 ) {
		return('l');	// left side
	} else {
		return('x');
	}

}

// ------------------------------------------------------------------- //

} /* namespace fuzz */
} /* namespace sfm */
