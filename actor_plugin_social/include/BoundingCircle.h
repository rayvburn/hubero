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
	void setRadius(const double &_radius);
	void setCenter(const ignition::math::Vector3d &_center_point);
	ignition::math::Vector3d getIntersection(ignition::math::Vector3d &_pt_dest);

	virtual ~BoundingCircle();

private:

	double radius;
	ignition::math::Vector3d center;

};

} /* namespace ActorUtils */

#endif /* INCLUDE_BOUNDINGCIRCLE_H_ */
