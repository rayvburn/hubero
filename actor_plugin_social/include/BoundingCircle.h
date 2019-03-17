/*
 * BoundingCircle.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_BOUNDINGCIRCLE_H_
#define INCLUDE_BOUNDINGCIRCLE_H_

#include <ignition/math/Vector3.hh>

namespace ActorUtils {

class BoundingCircle {

public:

	BoundingCircle();
	void SetRadius(const double &_radius);
	void SetCenter(const ignition::math::Vector3d &_center_point);
	ignition::math::Vector3d GetIntersection(ignition::math::Vector3d &_pt_dest);

	virtual ~BoundingCircle();

private:

	double radius;
	ignition::math::Vector3d center;

};

} /* namespace ActorUtils */

#endif /* INCLUDE_BOUNDINGCIRCLE_H_ */
