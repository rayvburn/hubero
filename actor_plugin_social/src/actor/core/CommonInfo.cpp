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

std::vector<std::shared_ptr<actor::inflation::Border> > 	CommonInfo::bounding_vector_;
std::vector<ignition::math::Vector3d>  						CommonInfo::lin_vel_vector_;
std::map<std::string, unsigned int>    						CommonInfo::name_id_map_;

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
	bounding_vector_.push_back(nullptr);
	name_id_map_.insert(std::make_pair(name, id_actor_));

}

// ------------------------------------------------------------------- //

void CommonInfo::setBorderPtr(std::shared_ptr<actor::inflation::Border> border_ptr) {
	bounding_vector_.at(id_actor_) = border_ptr;
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

std::shared_ptr<actor::inflation::Border> CommonInfo::getBorderPtr() const {
	return (bounding_vector_.at(id_actor_));
}

// ------------------------------------------------------------------- //

ignition::math::Vector3d CommonInfo::getLinearVelocity() const {
	return (lin_vel_vector_.at(id_actor_));
}

// ------------------------------------------------------------------- //

std::vector<std::shared_ptr<actor::inflation::Border> > CommonInfo::getBorderPtrsVector() const {
	return (bounding_vector_);
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
	bounding_vector_.at(id_actor_) = nullptr;

	//name_id_map_.clear();

}

// ------------------------------------------------------------------- //

CommonInfo::~CommonInfo() {
	clearInternalMemory();
}

// ------------------------------------------------------------------- //

} /* namespace core */
} /* namespace actor */
