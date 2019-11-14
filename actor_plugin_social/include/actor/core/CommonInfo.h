/*
 * CommonInfo.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_COMMONINFO_H_
#define INCLUDE_ACTOR_CORE_COMMONINFO_H_

#include <actor/inflation/Box.h>
#include <actor/inflation/Circle.h>
#include <actor/inflation/Ellipse.h>
#include <ignition/math/Vector3.hh>

#include <vector>
#include <string>
#include <map>

namespace actor {
namespace core {

/*
 * Private objects are made static here because whole info vectors must be passed to SFM.
 * Some info could not be set (at least in Gazebo 8.6.0) so actors can't have velocity,
 * acceleration, bounding boxes set as a common world properties thus properties
 * couldn't be read from the world pointer.
 */

// http://answers.gazebosim.org/question/22114/actor-related-information-in-gazebophysicsworldptr-and-collision-of-actors/

/*
 * Setting the linear velocity for actor or actor's model HAS NO EFFECT!
 * couldn't find source files on disk and based on bitbucket's Gazebo sources all seems
 * to be fine (32 joints detected so link pointer is not NULL)
 * Thus a workaround with static std::vector that stores all actor's velocities (and more)
 */

/*
 * Setting the linear velocity for actor or actor's model HAS NO EFFECT, don't know why,
 * couldn't find source files on disk and based on bitbucket's Gazebo sources all seems
 * to be fine (32 joints detected so link pointer is not NULL)
 * Thus a workaround with static std::vector that stores all actor's velocities
 */

/*
 * Actor has no collision thus no bounding box could be defined - it will be done artificially.
 */

/*
 * Bounding types could be switched dynamically if needed (if previously properly initialized)
 */

/// brief A class created as a workaround for
/// ActorPlugin problem related with not saving
/// a velocity, an acceleration, a bounding box;
/// it stores all actors' data that could not
/// be saved in the world file
class CommonInfo {

public:

	/// \brief Default constructor
	CommonInfo();

	/// \brief Method that assigns an ID for the actor
	/// that is invoked by (must be called for each actor);
	/// updates a private std::map instance which store
	/// an actor's name and his ID
	void addActor 			(const std::string &name);

	/// \brief Methods that update appropriate instance
	/// in a whole vector
	void setBoundingBox		(const actor::inflation::Box &bb);
	void setBoundingCircle	(const actor::inflation::Circle &bc);
	void setBoundingEllipse	(const actor::inflation::Ellipse &be);
	void setLinearVel		(const ignition::math::Vector3d &vel);

	unsigned int							getActorID() const;
	actor::inflation::Box					getBoundingBox() const;
	actor::inflation::Circle				getBoundingCircle() const;
	actor::inflation::Ellipse				getBoundingEllipse() const;
	ignition::math::Vector3d				getLinearVelocity() const;

	std::vector<actor::inflation::Box> 		getBoundingBoxesVector() const;
	std::vector<actor::inflation::Circle> 	getBoundingCirclesVector() const;
	std::vector<actor::inflation::Ellipse> 	getBoundingEllipsesVector() const;
	std::vector<ignition::math::Vector3d>	getLinearVelocitiesVector() const;
	std::map<std::string, unsigned int>		getNameIDMap() const;

	/// \brief Default destructor
	virtual ~CommonInfo();

private:

	/// \brief Helper function that clears all static
	/// instances of a class
	void clearInternalMemory();

	/// \brief Actor's ID for indexing the vectors;
	/// this is related to an actor which class is
	/// constructed for
	unsigned int id_actor_;

	/// \brief Static vectors of a class which store
	/// instances of a given type for whole actors
	/// population - length of a vector is equal
	/// actors number in a world
	static std::vector<actor::inflation::Box> 	  bounding_box_vector_;
	static std::vector<actor::inflation::Circle>  bounding_circle_vector_;
	static std::vector<actor::inflation::Ellipse> bounding_ellipse_vector_;
	static std::vector<ignition::math::Vector3d>  lin_vel_vector_;

	/// \brief A map which stores actors names
	/// and assigns their corresponding IDs each time
	/// addActor() is invoked; thus addActor() must
	/// be called first
	static std::map<std::string, unsigned int>    name_id_map_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_COMMONINFO_H_ */
