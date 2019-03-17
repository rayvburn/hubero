/*
 * CommonInfo.cpp
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#include <CommonInfo.h>

namespace ActorUtils {

// ------------------------------------------------------------------- //

std::vector<ignition::math::Box> 	  	CommonInfo::bounding_box_vector;
std::vector<BoundingCircle> 			CommonInfo::bounding_circle_vector;
std::vector<ignition::math::Vector3d>  	CommonInfo::lin_vel_vector;
std::map<std::string, unsigned int>    	CommonInfo::name_id_map;

// ------------------------------------------------------------------- //

CommonInfo::CommonInfo():
	id(0)
{
	// as data are static they will be cleared after 2nd actor's constructor execution
	// clearInternalMemory();
}

// ------------------------------------------------------------------- //

void CommonInfo::AddActor (const std::string &_name) {

	lin_vel_vector.emplace_back(1.0, 1.0, 0.0);
	this->id = static_cast<unsigned int>(lin_vel_vector.size() - 1);
	bounding_box_vector.push_back(ignition::math::Box(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	bounding_circle_vector.push_back(BoundingCircle());
	name_id_map.insert(std::make_pair(_name, this->id));

}

// ------------------------------------------------------------------- //

void CommonInfo::SetBoundingBox (const ignition::math::Box &_bb) {
	bounding_box_vector.at(this->id) = _bb;
}

// ------------------------------------------------------------------- //

void CommonInfo::SetBoundingCircle (const BoundingCircle &_bc) {
	bounding_circle_vector.at(this->id) = _bc;
}

// ------------------------------------------------------------------- //

void CommonInfo::SetLinearVel (const ignition::math::Vector3d &_vel) {
	lin_vel_vector.at(this->id) = _vel;
}

// ------------------------------------------------------------------- //

unsigned int CommonInfo::GetActorID() const {
	return (this->id);
}

// ------------------------------------------------------------------- //

ignition::math::Box	CommonInfo::GetBoundingBox() const {
	return (bounding_box_vector.at(this->id));
}

// ------------------------------------------------------------------- //

BoundingCircle CommonInfo::GetBoundingCircle() const {
	return (bounding_circle_vector.at(this->id));
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d CommonInfo::GetLinearVelocity() const {
	return (lin_vel_vector.at(this->id));
}

// ------------------------------------------------------------------- //

std::vector<ignition::math::Box> CommonInfo::GetBoundingBoxesVector() const {
	return (bounding_box_vector);
}

// ------------------------------------------------------------------- //

std::vector<BoundingCircle> CommonInfo::GetBoundingCirclesVector() const {
	return (bounding_circle_vector);
}

// ------------------------------------------------------------------- //

std::vector<ignition::math::Vector3d> CommonInfo::GetLinearVelocitiesVector() const {
	return (lin_vel_vector);
}

// ------------------------------------------------------------------- //

std::map<std::string, unsigned int> CommonInfo::GetNameIDMap() const {
	return (name_id_map);
}

// ------------------------------------------------------------------- //

void CommonInfo::ClearInternalMemory() {

	lin_vel_vector.clear();
	bounding_box_vector.clear();
	bounding_circle_vector.clear();
	name_id_map.clear();

}

// ------------------------------------------------------------------- //

CommonInfo::~CommonInfo() {
	ClearInternalMemory();
}

} /* namespace ActorUtils */
