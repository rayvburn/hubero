/*
 * CommonInfo.cpp
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#include <CommonInfo.h>

namespace ActorUtils {

// ------------------------------------------------------------------- //

std::vector<actor::inflation::Box> 	  	CommonInfo::bounding_box_vector;
std::vector<actor::inflation::Circle> 	CommonInfo::bounding_circle_vector;
std::vector<actor::inflation::Ellipse> 	CommonInfo::bounding_ellipse_vector;
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
	bounding_box_vector.push_back(actor::inflation::Box());
	bounding_circle_vector.push_back(actor::inflation::Circle());
	bounding_ellipse_vector.push_back(actor::inflation::Ellipse());
	name_id_map.insert(std::make_pair(_name, this->id));

}

// ------------------------------------------------------------------- //

void CommonInfo::SetBoundingBox (const actor::inflation::Box &_bb) {
	bounding_box_vector.at(this->id) = _bb;
}

// ------------------------------------------------------------------- //

void CommonInfo::SetBoundingCircle (const actor::inflation::Circle &_bc) {
	bounding_circle_vector.at(this->id) = _bc;
}

// ------------------------------------------------------------------- //

void CommonInfo::SetBoundingEllipse	(const actor::inflation::Ellipse &_be) {
	bounding_ellipse_vector.at(this->id) = _be;
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

actor::inflation::Box CommonInfo::GetBoundingBox() const {
	return (bounding_box_vector.at(this->id));
}

// ------------------------------------------------------------------- //

actor::inflation::Circle CommonInfo::GetBoundingCircle() const {
	return (bounding_circle_vector.at(this->id));
}

// ------------------------------------------------------------------- //

actor::inflation::Ellipse CommonInfo::GetBoundingEllipse() const {
	return (bounding_ellipse_vector.at(this->id));
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d CommonInfo::GetLinearVelocity() const {
	return (lin_vel_vector.at(this->id));
}

// ------------------------------------------------------------------- //

std::vector<actor::inflation::Box> CommonInfo::GetBoundingBoxesVector() const {
	return (bounding_box_vector);
}

// ------------------------------------------------------------------- //

std::vector<actor::inflation::Circle> CommonInfo::GetBoundingCirclesVector() const {
	return (bounding_circle_vector);
}

// ------------------------------------------------------------------- //

std::vector<actor::inflation::Ellipse> CommonInfo::GetBoundingEllipsesVector() const {
	return (bounding_ellipse_vector);
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
	bounding_ellipse_vector.clear();
	name_id_map.clear();

}

// ------------------------------------------------------------------- //

CommonInfo::~CommonInfo() {
	ClearInternalMemory();
}

} /* namespace ActorUtils */
