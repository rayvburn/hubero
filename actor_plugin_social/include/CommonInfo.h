/*
 * CommonInfo.h
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_COMMONINFO_H_
#define INCLUDE_COMMONINFO_H_

#include <ignition/math/Box.hh>
#include <ignition/math/Vector3.hh>
#include "BoundingCircle.h"
#include "Enums.h"
#include <vector>
#include <string>
#include <map>

namespace ActorUtils {

/*
 * Private objects are made static here because whole info vectors must be passed to SFM.
 * Some info could not be set (at least in Gazebo8) and actors can't have velocity,
 * acceleration, bounding boxes set as a common world properties thus properties
 * couldn't be read from the world pointer.
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
	void addActor 			(const std::string &_name);

	void setBoundingBox		(const ignition::math::Box &_bb);
	void setBoundingCircle	(const BoundingCircle &_bc);
	void setLinearVel		(const ignition::math::Vector3d &_vel);

	unsigned int							getActorID() const;
	ignition::math::Box						getBoundingBox() const;
	BoundingCircle							getBoundingCircle() const;
	ignition::math::Vector3d				getLinearVelocity() const;

	std::vector<ignition::math::Box> 		getBoundingBoxesVector() const;
	std::vector<BoundingCircle> 	 		getBoundingCirclesVector() const;
	std::vector<ignition::math::Vector3d>	getLinearVelocitiesVector() const;
	std::map<std::string, unsigned int>		getNameIDMap() const;

	virtual ~CommonInfo();

private:

	void clearInternalMemory();

	/// \brief Actor's ID for indexing the vectors
	unsigned int id;

	static std::vector<ignition::math::Box> 	  bounding_box_vector;
	static std::vector<BoundingCircle> 			  bounding_circle_vector;
	static std::vector<ignition::math::Vector3d>  lin_vel_vector;
	static std::map<std::string, unsigned int>    name_id_map;

};

} /* namespace ActorUtils */

#endif /* INCLUDE_COMMONINFO_H_ */
