/*
 * ActorInfoDecoder.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#include <sfm/core/ActorInfoDecoder.h>
#include "core/Enums.h" // ACTOR_MODEL_TYPE_ID

namespace sfm {
namespace core {


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

	if ( model_type == actor::ACTOR_MODEL_TYPE_ID ) {
		return (true);
	} else {
		return (false);
	}

}

// ------------------------------------------------------------------- //

ActorInfoDecoder::~ActorInfoDecoder() { }

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace sfm */
