/*
 * CommonInfo.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_COMMONINFO_H_
#define INCLUDE_ACTOR_CORE_COMMONINFO_H_

#include <ignition/math/Vector3.hh>

#include "inflation/Ellipse.h"
#include "inflation/Circle.h"
#include "inflation/Box.h"

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
 * Bounding types could be switched dynamically if needed
 */

class CommonInfo {

public:

	CommonInfo();

	/// \brief Method that assigns an ID for the actor that is invoked by (must be called for each actor)
	void addActor 			(const std::string &name);

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

	virtual ~CommonInfo();

private:

	void clearInternalMemory();

	/// \brief Actor's ID for indexing the vectors
	unsigned int id_actor_;

	static std::vector<actor::inflation::Box> 	  bounding_box_vector_;
	static std::vector<actor::inflation::Circle>  bounding_circle_vector_;
	static std::vector<actor::inflation::Ellipse> bounding_ellipse_vector_;
	static std::vector<ignition::math::Vector3d>  lin_vel_vector_;
	static std::map<std::string, unsigned int>    name_id_map_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_COMMONINFO_H_ */
