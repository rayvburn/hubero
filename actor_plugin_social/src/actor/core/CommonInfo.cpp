/*
 * CommonInfo.cpp
 *
 *  Created on: Mar 16, 2019
 *      Author: rayvburn
 */

#include <actor/core/CommonInfo.h>

namespace actor {
namespace core {

// ------------------------------------------------------------------- //

std::vector<actor::inflation::Box> 	  	CommonInfo::bounding_box_vector_;
std::vector<actor::inflation::Circle> 	CommonInfo::bounding_circle_vector_;
std::vector<actor::inflation::Ellipse> 	CommonInfo::bounding_ellipse_vector_;
std::vector<ignition::math::Vector3d>  	CommonInfo::lin_vel_vector_;
std::map<std::string, unsigned int>    	CommonInfo::name_id_map_;

// ------------------------------------------------------------------- //

CommonInfo::CommonInfo():
	id_actor_(0)
{
	// as data are static they will be cleared after 2nd actor's constructor execution
	// clearInternalMemory();
}

// ------------------------------------------------------------------- //

void CommonInfo::addActor (const std::string &name) {

	lin_vel_vector_.emplace_back(0.0, 0.0, 0.0);
	id_actor_ = static_cast<unsigned int>(lin_vel_vector_.size() - 1);
	bounding_box_vector_.push_back(actor::inflation::Box());
	bounding_circle_vector_.push_back(actor::inflation::Circle());
	bounding_ellipse_vector_.push_back(actor::inflation::Ellipse());
	name_id_map_.insert(std::make_pair(name, id_actor_));

}

// ------------------------------------------------------------------- //

void CommonInfo::setBoundingBox (const actor::inflation::Box &bb) {
	bounding_box_vector_.at(id_actor_) = bb;
}

// ------------------------------------------------------------------- //

void CommonInfo::setBoundingCircle (const actor::inflation::Circle &bc) {
	bounding_circle_vector_.at(id_actor_) = bc;
}

// ------------------------------------------------------------------- //

void CommonInfo::setBoundingEllipse	(const actor::inflation::Ellipse &be) {
	bounding_ellipse_vector_.at(id_actor_) = be;
}

// ------------------------------------------------------------------- //

void CommonInfo::setLinearVel (const ignition::math::Vector3d &vel) {
	lin_vel_vector_.at(id_actor_) = vel;
}

// ------------------------------------------------------------------- //

unsigned int CommonInfo::getActorID() const {
	return (id_actor_);
}

// ------------------------------------------------------------------- //

actor::inflation::Box CommonInfo::getBoundingBox() const {
	return (bounding_box_vector_.at(id_actor_));
}

// ------------------------------------------------------------------- //

actor::inflation::Circle CommonInfo::getBoundingCircle() const {
	return (bounding_circle_vector_.at(id_actor_));
}

// ------------------------------------------------------------------- //

actor::inflation::Ellipse CommonInfo::getBoundingEllipse() const {
	return (bounding_ellipse_vector_.at(id_actor_));
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d CommonInfo::getLinearVelocity() const {
	return (lin_vel_vector_.at(id_actor_));
}

// ------------------------------------------------------------------- //

std::vector<actor::inflation::Box> CommonInfo::getBoundingBoxesVector() const {
	return (bounding_box_vector_);
}

// ------------------------------------------------------------------- //

std::vector<actor::inflation::Circle> CommonInfo::getBoundingCirclesVector() const {
	return (bounding_circle_vector_);
}

// ------------------------------------------------------------------- //

std::vector<actor::inflation::Ellipse> CommonInfo::getBoundingEllipsesVector() const {
	return (bounding_ellipse_vector_);
}

// ------------------------------------------------------------------- //

std::vector<ignition::math::Vector3d> CommonInfo::getLinearVelocitiesVector() const {
	return (lin_vel_vector_);
}

// ------------------------------------------------------------------- //

std::map<std::string, unsigned int> CommonInfo::getNameIDMap() const {
	return (name_id_map_);
}

// ------------------------------------------------------------------- //

void CommonInfo::clearInternalMemory() {

	lin_vel_vector_.clear();
	bounding_box_vector_.clear();
	bounding_circle_vector_.clear();
	bounding_ellipse_vector_.clear();
	name_id_map_.clear();

}

// ------------------------------------------------------------------- //

CommonInfo::~CommonInfo() {
	clearInternalMemory();
}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */