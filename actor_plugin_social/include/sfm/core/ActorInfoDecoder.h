/*
 * ActorInfoDecoder.h
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_CORE_ACTORINFODECODER_H_
#define INCLUDE_SFM_CORE_ACTORINFODECODER_H_

#include <ignition/math/Vector3.hh>
#include <vector>
#include <string>
#include <map>

#include "inflation/Box.h"
#include "inflation/Circle.h"
#include "inflation/Ellipse.h"

namespace sfm {
namespace core {

/// more of a failure blocker that real decoder -
/// decoder returns default object constructor when actor is not found
class ActorInfoDecoder {

public:

	ActorInfoDecoder();

	void 						setID(const std::string &name, const std::map<std::string, unsigned int> &map);
	bool						isActor(const unsigned int &model_type) const;

	// TODO: make a template
	ignition::math::Vector3d 	getVelocity			(const std::vector<ignition::math::Vector3d> &actors_velocities) 			const;
	actor::inflation::Box 		getBoundingBox		(const std::vector<actor::inflation::Box> &actors_bounding_boxes) 		const;
	actor::inflation::Circle 	getBoundingCircle	(const std::vector<actor::inflation::Circle> &actors_bounding_circles) 	const;
	actor::inflation::Ellipse 	getBoundingEllipse	(const std::vector<actor::inflation::Ellipse> &actors_bounding_ellipses) 	const;

	virtual ~ActorInfoDecoder();

private:

	unsigned int id_actor_;

};

} /* namespace core */
} /* namespace sfm */

#endif /* INCLUDE_SFM_CORE_ACTORINFODECODER_H_ */
