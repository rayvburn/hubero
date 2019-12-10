/*
 * WorldBoundary.h
 *
 *  Created on: Dec 10, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_CORE_WORLDBOUNDARY_H_
#define INCLUDE_SFM_CORE_WORLDBOUNDARY_H_

#include <actor/ros_interface/ParamLoader.h>
#include <gazebo/physics/World.hh>
#include <ignition/math/Box.hh>

namespace sfm {
namespace core {

/** \section WorldBoundary class may not be needed if the `world boundaries` (i.e. walls)
 * have been created from separate models - walls pointers are null then.
 */
class WorldBoundary {

public:

	/// \brief Default constructor
	WorldBoundary();

	/// \brief Initializes class internal state and parameters
	bool init(std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr, const gazebo::physics::WorldPtr &world_ptr);

	/// \brief Evaluates validity of the wall associated with the `wall_id`.
	/// \param wall_id: is a number of wall (0-3)
	/// \return True is BoundingBox pointer is not null
	bool isValid(const size_t &wall_id) const;

	/// \brief Returns a pointer to a single wall bounding box. The pointer cannot be modified.
	/// \param wall_id: is a number of wall (0-3)
	/// \return Pointer to constant `ignition::math::Box` instance
	const ignition::math::Box* getBoundingBoxPtr(const size_t &wall_id) const;

	/// \brief Destructor
	virtual ~WorldBoundary();

private:

	/// \brief Performs a division of world model (in combined form)
	/// into 4 separate bounding boxes.
	/// \return True if the operation was successful
	bool divide(const ignition::math::Box &bb, const double &wall_width);

	/// \brief Shared pointer to ParamLoader class.
	std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr_;

	/// \brief Bounding boxes expressed as pointers so they can be easily evaluated if are valid models.
	std::vector<ignition::math::Box*> box_ptr_v_;

};

} /* namespace core */
} /* namespace sfm */

#endif /* INCLUDE_SFM_CORE_WORLDBOUNDARY_H_ */
