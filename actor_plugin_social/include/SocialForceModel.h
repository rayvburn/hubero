/*
 * SocialForceModel.h
 *
 *  Created on: Dec 4, 2018
 *      Author: rayvburn
 */

#ifndef INCLUDE_SOCIALFORCEMODEL_H_
#define INCLUDE_SOCIALFORCEMODEL_H_

#include <ignition/math.hh>

/* Calculations are based on:
 * 		- D. Helbing's - Social Force Model for Pedestrian Dynamics ‎(1998)
 * 		- Moussaid et al., 2009
 */

namespace SocialForceModel {

enum ObjectType {
	STATIC_REPULSIVE = 0,
	DYNAMIC_REPULSIVE,
	STATIC_ATTRACTIVE,
	DYNAMIC_ATTRACTIVE
};

class SocialForceModel {

public:

	SocialForceModel();
	double GetSocialForce();
	virtual ~SocialForceModel();

private:

	double GetInternalAcceleration(ObjectType _type);
	double GetRepulsion(ObjectType _type);
	double GetAttraction(ObjectType _type);
	ignition::math::Vector3d CalculateNumGradient(ignition::math::Vector3d &_vector);

	float RELAXATION_TIME;
	float DESIRED_SPEED;

};

}

#endif /* INCLUDE_SOCIALFORCEMODEL_H_ */
