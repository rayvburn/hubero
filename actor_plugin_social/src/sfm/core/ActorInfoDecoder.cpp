/*
 * ActorInfoDecoder.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#include <sfm/core/ActorInfoDecoder.h>

namespace sfm {
namespace core {

static const unsigned int ACTOR_NOT_FOUND = 65534;
static const unsigned int ACTOR_MODEL_ID = 32771;

// ------------------------------------------------------------------- //

ActorInfoDecoder::ActorInfoDecoder(): id_actor_(ACTOR_NOT_FOUND) { }

// ------------------------------------------------------------------- //

void ActorInfoDecoder::setID(const std::string &name, const std::map<std::string, unsigned int> &map) {

	std::map<std::string, unsigned int>::const_iterator it;
	it = map.find(name);

	// iterator pointing to end of a map indicates NOT FOUND
	if ( it != map.end() ) {
		id_actor_ = it->second;
	} else {
		id_actor_ = ACTOR_NOT_FOUND;
	}

}

// ------------------------------------------------------------------- //

bool ActorInfoDecoder::isActor(const unsigned int &model_type) const {

	if ( model_type == ACTOR_MODEL_ID ) {
		return (true);
	} else {
		return (false);
	}

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d ActorInfoDecoder::getVelocity(const std::vector<ignition::math::Vector3d> &actors_velocities) const {

	if ( id_actor_ == ACTOR_NOT_FOUND ) {
		return (ignition::math::Vector3d(0.0, 0.0, 0.0));
	}
	return (actors_velocities[id_actor_]);

}

// ------------------------------------------------------------------- //

actor::inflation::Box ActorInfoDecoder::getBoundingBox(const std::vector<actor::inflation::Box> &actors_bounding_boxes) const {

	if ( id_actor_ == ACTOR_NOT_FOUND ) {
		return (actor::inflation::Box());
	}
	return (actors_bounding_boxes[id_actor_]);

}

// ------------------------------------------------------------------- //

actor::inflation::Circle ActorInfoDecoder::getBoundingCircle(const std::vector<actor::inflation::Circle> &actors_bounding_circles) const {

	if ( id_actor_ == ACTOR_NOT_FOUND ) {
		return (actor::inflation::Circle());
	}
	return (actors_bounding_circles[id_actor_]);

}

// ------------------------------------------------------------------- //

actor::inflation::Ellipse ActorInfoDecoder::getBoundingEllipse(const std::vector<actor::inflation::Ellipse> &actors_bounding_ellipses) const {

	if ( id_actor_ == ACTOR_NOT_FOUND ) {
		return (actor::inflation::Ellipse());
	}
	return (actors_bounding_ellipses[id_actor_]);

}

// ------------------------------------------------------------------- //

ActorInfoDecoder::~ActorInfoDecoder() { }

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace sfm */
