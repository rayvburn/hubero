/*
 * WorldBoundary.cpp
 *
 *  Created on: Dec 10, 2019
 *      Author: rayvburn
 */

#include <sfm/core/WorldBoundary.h>
#include <gazebo/physics/Model.hh>

namespace sfm {
namespace core {

// ------------------------------------------------------------------- //

WorldBoundary::WorldBoundary() {
	params_ptr_ = nullptr;
	// fill BB vector with null pointers
	for ( size_t i = 0; i < 4; i++ ) {
		box_ptr_v_.push_back(nullptr);
	}
}

// ------------------------------------------------------------------- //

bool WorldBoundary::init(std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr,
						 const gazebo::physics::WorldPtr &world_ptr) {

	params_ptr_ = params_ptr;
//	while (!world_ptr->IsLoaded());
	gazebo::physics::ModelPtr model_ptr = world_ptr->ModelByName(params_ptr_->getSfmDictionary().world_model.name);
	if ( model_ptr == nullptr ) {
		return (false);
	}

	std::cout << "\n\n\n[WorldBoundary::init] model: " << model_ptr->GetName() << "\t";
	std::cout << "center: " << model_ptr->BoundingBox().Center() << "\t";
	std::cout << "min: " << model_ptr->BoundingBox().Min() << "\t";
	std::cout << "max: " << model_ptr->BoundingBox().Max() << "\n\n\n";

	return (divide(model_ptr->BoundingBox(), params_ptr_->getSfmDictionary().world_model.wall_width));

}

// ------------------------------------------------------------------- //

bool WorldBoundary::isValid(const size_t &wall_id) const {

	// indexes from the range of 0-3 (inclusive)
	if ( wall_id < 0 || wall_id > 3 ) {
		return (false);
	}

	// check if pointer is null
	if ( box_ptr_v_.at(wall_id) != nullptr ) {
		return (true);
	}

	return (false);

}

// ------------------------------------------------------------------- //

const ignition::math::Box* WorldBoundary::getBoundingBoxPtr(const size_t &wall_id) const {
	return (box_ptr_v_.at(wall_id));
}

// ------------------------------------------------------------------- //

WorldBoundary::~WorldBoundary() {

	for ( size_t i = 0; i < 4; i++ ) {
		delete (box_ptr_v_.at(i));
	}

}

// ------------------------------------------------------------------- //

bool WorldBoundary::divide(const ignition::math::Box &bb, const double &wall_width) {

	// Bounding box division considering wall width given by a parameter;
	// notes on calculation method in the notebook (10.12.2019)

	// 1st box
	ignition::math::Box* bb1_ptr = new ignition::math::Box(bb.Max().X(), 				bb.Max().Y(), 				bb.Max().Z(), 	// x1, y1, z1
														   bb.Max().X() - wall_width, 	bb.Min().Y(), 				bb.Min().Z()); 	// x2, y2, z2
	// 2nd box
	ignition::math::Box* bb2_ptr = new ignition::math::Box(bb.Max().X(), 				bb.Min().Y() + wall_width, 	bb.Max().Z(),
														   bb.Min().X(), 				bb.Min().Y(), 				bb.Min().Z());
	// 3rd box
	ignition::math::Box* bb3_ptr = new ignition::math::Box(bb.Min().X() + wall_width, 	bb.Max().Y(), 				bb.Max().Z(),
														   bb.Min().X(), 				bb.Min().Y(), 				bb.Min().Z());
	// 4th box
	ignition::math::Box* bb4_ptr = new ignition::math::Box(bb.Max().X(), 				bb.Max().Y(), 				bb.Max().Z(),
														   bb.Min().X(), 				bb.Max().Y() - wall_width, 	bb.Min().Z());

	std::cout << "\t1) center: " << bb1_ptr->Center() << "\tMin: " << bb1_ptr->Min() << "\tMax: " << bb1_ptr->Max() << std::endl;
	std::cout << "\t2) center: " << bb2_ptr->Center() << "\tMin: " << bb2_ptr->Min() << "\tMax: " << bb2_ptr->Max() << std::endl;
	std::cout << "\t3) center: " << bb3_ptr->Center() << "\tMin: " << bb3_ptr->Min() << "\tMax: " << bb3_ptr->Max() << std::endl;
	std::cout << "\t4) center: " << bb4_ptr->Center() << "\tMin: " << bb4_ptr->Min() << "\tMax: " << bb4_ptr->Max() << std::endl;

	// update box vector
	box_ptr_v_.at(0) = bb1_ptr;
	box_ptr_v_.at(1) = bb2_ptr;
	box_ptr_v_.at(2) = bb3_ptr;
	box_ptr_v_.at(3) = bb4_ptr;

	return (true);

}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace sfm */
