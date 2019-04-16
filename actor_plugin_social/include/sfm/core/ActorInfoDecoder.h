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

/// more of a failure blocker than a real decoder -
/// decoder returns default object constructor when actor is not found
class ActorInfoDecoder {

public:

	/// \brief Default constructor - this class is more
	/// of a failure blocker than a real decoder -
	/// it manages actor::core::CommonInfo's vectors
	/// to make SFM use some information which could
	/// not be passed to Gazebo's WorldPtr and which
	/// are necessary for SFM computations
	ActorInfoDecoder();

	/// \brief Sets ID based on an actor's name
	/// according to std::map provided
	void setID(const std::string &name, const std::map<std::string, unsigned int> &map);

	/// \brief Based on a Gazebo model type checks
	/// whether an object is an actor
	bool isActor(const unsigned int &model_type) const;

	/// \brief Default destructor
	virtual ~ActorInfoDecoder();

private:

	/// \brief Default value for id_actor
	static constexpr unsigned int ACTOR_NOT_FOUND = 65534;

	/// \brief Gazebo model ID for ActorPlugin obejcts
	static constexpr unsigned int ACTOR_MODEL_ID = 32771;

	/// \brief ID of an actor, changes as SFM is computed
	/// for consecutive actors
	unsigned int id_actor_;

public:

	/// \brief Template that manages objects in a given vector
	/// based on ID which must be set previously
	/// \return If actor is found - returns appropriate object
	/// from a given vector based on ID; otherwise - a default
	/// constructor
	template <typename T>
	T getData(const std::vector<T> &vector) const {

		if ( id_actor_ == ACTOR_NOT_FOUND ) {
			// return default constructor
			return ( T() );
		}
		return ( vector.at(id_actor_) );

	}

};

} /* namespace core */
} /* namespace sfm */

#endif /* INCLUDE_SFM_CORE_ACTORINFODECODER_H_ */
