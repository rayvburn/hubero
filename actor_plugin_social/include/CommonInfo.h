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
#include "BoundingEllipse.h"

#include "inflation/Ellipse.h"
#include "inflation/Circle.h"
#include "inflation/Box.h"

#include "Enums.h"
#include <vector>
#include <string>
#include <map>

namespace ActorUtils {

/*
 * Private objects are made static here because whole info vectors must be passed to SFM.
 * Some info could not be set (at least in Gazebo 8.6.0) so actors can't have velocity,
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
	void AddActor 			(const std::string &_name);

	void SetBoundingBox		(const actor::inflation::Box &_bb);
	void SetBoundingCircle	(const actor::inflation::Circle &_bc);
	void SetBoundingEllipse	(const actor::inflation::Ellipse &_be);
	void SetLinearVel		(const ignition::math::Vector3d &_vel);

	unsigned int							GetActorID() const;
	actor::inflation::Box					GetBoundingBox() const;
	actor::inflation::Circle				GetBoundingCircle() const;
	actor::inflation::Ellipse				GetBoundingEllipse() const;
	ignition::math::Vector3d				GetLinearVelocity() const;

	std::vector<actor::inflation::Box> 		GetBoundingBoxesVector() const;
	std::vector<actor::inflation::Circle> 	GetBoundingCirclesVector() const;
	std::vector<actor::inflation::Ellipse> 	GetBoundingEllipsesVector() const;
	std::vector<ignition::math::Vector3d>	GetLinearVelocitiesVector() const;
	std::map<std::string, unsigned int>		GetNameIDMap() const;

	virtual ~CommonInfo();

private:

	void ClearInternalMemory();

	/// \brief Actor's ID for indexing the vectors
	unsigned int id;

	static std::vector<actor::inflation::Box> 	  bounding_box_vector;
	static std::vector<actor::inflation::Circle>  bounding_circle_vector;
	static std::vector<actor::inflation::Ellipse> bounding_ellipse_vector;
	static std::vector<ignition::math::Vector3d>  lin_vel_vector;
	static std::map<std::string, unsigned int>    name_id_map;

};

} /* namespace ActorUtils */

#endif /* INCLUDE_COMMONINFO_H_ */
