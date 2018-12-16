/*
 * SocialForceModel.cpp
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#include <SocialForceModel.h>

namespace SocialForceModel {

// ------------------------------------------------------------------- //

SocialForceModel::SocialForceModel():

	speed_max(1.5) {

	SetParameterValues();

	/* Algorithm PARAMETERS are:
	 * - relaxation time must be given here
	 * - kind of coefficient for attraction artificial potential field (decreases over time)
	 * - FOV
	 * - direction weight
	 * - max speed
	 */

}

// ------------------------------------------------------------------- //

double SocialForceModel::GetSocialForce() {

	/* Algorithm INPUTS are:
	 * - dt
	 * - goal reaching component (acceleration term)
	 * 		- target's pose
	 * 		- current actor's pose
	 * 		- actual speed
	 * 		- desired speed
	 * 		- relaxation time
	 * - dynamic object repulsion component (other people or obstacles)
	 * 		- object's pose
	 * 		- current actor's pose
	 * 		- object's speed
	 * - static object repulsion component (borders)
	 * 		- object's pose
	 * 		- current actor's pose
	 *	- object attraction component (other people or objects)
	 *		- field decrease factor
	 *		- object's pose
	 *		- current actor's pose
	 *
	 */

	return 0.00;

}

// ------------------------------------------------------------------- //

void SocialForceModel::SetParameterValues() {

	/* Invoking this function to each actor will create a population in which there are
	 * everyone moving in slightly other way */

	std::default_random_engine rand_gen;	// random number generator

	// desired speed (based on (Moussaid et al. (2009))
	std::normal_distribution<float> dist_spd_desired(1.29F, 0.19F);
	speed_desired = dist_spd_desired(rand_gen);

	// relaxation time (based on (Moussaid et al. (2009))
	std::normal_distribution<float> dist_tau(0.54F, 0.05F);
	relaxation_time = dist_tau(rand_gen);

	// ----------------------------- Model C ------------------------------------------------------ //
	// Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014)
	// Generate random value of mean a and standard deviation b
	std::normal_distribution<float> dist_an(0.2615F, 0.0551F);		An = dist_an(rand_gen);
	std::normal_distribution<float> dist_bn(0.4026F, 0.1238F);		Bn = dist_bn(rand_gen);
	std::normal_distribution<float> dist_cn(2.1614F, 0.3728F);		Cn = dist_cn(rand_gen);
	std::normal_distribution<float> dist_ap(1.5375F, 0.3084F);		Ap = dist_ap(rand_gen);
	std::normal_distribution<float> dist_bp(0.4938F, 0.1041F);		Bp = dist_bp(rand_gen);
	std::normal_distribution<float> dist_cp(0.5710F, 0.1409F);		Cp = dist_cp(rand_gen);
	std::normal_distribution<float> dist_aw(0.3280F, 0.1481F);		Aw = dist_aw(rand_gen);
	std::normal_distribution<float> dist_bw(0.1871F, 0.0563F);		Bw = dist_bw(rand_gen);

}

// ------------------------------------------------------------------- //

/* DEPRECATED
ignition::math::Vector3d SocialForceModel::CalculateNumGradient(ignition::math::Vector3d &_vector) {

	// Vector3D is just for convenience (to avoid conversion)
	// for 2 dimensional vectors numerical gradient is just a difference (y-x)
	ignition::math::Vector3d num_gradient(_vector.Y()-_vector.X(), _vector.Y()-_vector.X(), 0.0);
	return num_gradient;

}
*/

// ------------------------------------------------------------------- //

SocialForceModel::~SocialForceModel() { }

}
