/*
 * Types.h
 *
 *  Created on: Sep 26, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_TYPES_H_
#define INCLUDE_TYPES_H_

#include <ignition/math.hh>

// FIXME: force whole system to use those typedefs instead of `ignition::math::Vector3d` etc.

typedef ignition::math::Vector2d 	Vector2;
typedef ignition::math::Vector3d 	Vector3;
typedef ignition::math::Pose3d	 	Pose;
typedef ignition::math::Angle		Angle;
typedef ignition::math::Line2d		Line2;
typedef ignition::math::Line3d		Line3;
typedef ignition::math::Quaterniond Quaternion;
typedef ignition::math::Box			Box;

#endif /* INCLUDE_TYPES_H_ */
