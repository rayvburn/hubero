/*
 * SocialForceModel.h
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#ifndef INCLUDE_SOCIALFORCEMODEL_H_
#define INCLUDE_SOCIALFORCEMODEL_H_

#include <SocialForceModelUtils.h>
#include <ignition/math.hh> // ?? needed still

// ----------------------------------------------------------------------------------------------- //
/* References:
 * 		- D. Helbing et al. 	- Social Force Model for Pedestrian Dynamics ‎(1998)
 * 		- Moussaid et al. 		- Experimental study of the behavioural mechanisms underlying
 * 		  						  self-organization in human crowds (2009)
 * 		- S. Seer et al. 		- Validating Social Force based Models with Comprehensive
 * 		  						  Real World Motion Data (2014)
 *
 * https://www.sciencedirect.com/science/article/pii/S2352146514001161
 * In the above paper there are results of the research presented. They point that
 * Model C (Rudloff et al. (2011) of SFM fits best the real world data.
 */
// ----------------------------------------------------------------------------------------------- //

namespace SocialForceModel {

class SocialForceModel {

public:

	SocialForceModel();
	double GetSocialForce();
	virtual ~SocialForceModel();

private:

	void 	SetParameterValues(void);
	double 	GetInternalAcceleration(SFMObjectType _type);
	double 	GetRepulsion(SFMObjectType _type);
	double 	GetAttraction(SFMObjectType _type);

	float relaxation_time;
	float speed_desired;
	float speed_max;

	// Rudloff et al. (2011) model's parameters based on  S. Seer et al. (2014)
	float An;
	float Bn;
	float Cn;
	float Ap;
	float Bp;
	float Cp;
	float Aw;
	float Bw;

	/* setting parameters static will create a population of actors moving in the same way */
	/*
	static float An;
	static float Bn;
	static float Cn;
	static float Ap;
	static float Bp;
	static float Cp;
	static float Aw;
	static float Bw;
	*/

};

}

#endif /* INCLUDE_SOCIALFORCEMODEL_H_ */
