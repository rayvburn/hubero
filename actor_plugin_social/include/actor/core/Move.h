/*
 * Move.h
 *
 *  Created on: Jan 3, 2020
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_MOVE_H_
#define INCLUDE_ACTOR_CORE_MOVE_H_

#include <actor/core/PTF.h>
#include <fuzz/Processor.h>
#include <fuzz/SocialConductor.h>
#include <actor/core/Path.h>
#include <actor/ros_interface/ParamLoader.h>
#include <sfm/core/SocialForceModel.h>
#include <actor/core/Velocity.h>

namespace actor {
namespace core {

class Move : public PTF {

public:

	Move();

	/// \brief Updates internal state according to a given structure's content
	/// \note fuzz::SocialConductor-related
//	void configure(const actor::ros_interface::ParamLoader::BehaviourParams &beh_params_ptr); // FIXME

	void configure(std::shared_ptr<Path> path_storage_ptr);
	void configure(std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr);
	void configure(std::shared_ptr<sfm::SocialForceModel> sfm_ptr);
	void configure(std::shared_ptr<const actor::core::CommonInfo> common_info_ptr);
	void configure(std::shared_ptr<const std::vector<std::string> >  ignored_models_ptr);
	void configure(gazebo::physics::WorldPtr world_ptr);
	void configure(std::shared_ptr<const actor::core::Velocity> velocity_ptr);

	virtual void execute() {};

	void execute(const double &dt);

	double getDisplacement() const { return (dist_traveled_); }

	/// \brief Returns the superposed social force vector.
	/// \return Superposed social force vector
	/// \note SocialConductor-related
	ignition::math::Vector3d getSocialVector() const;

	/// \brief Returns the last active behaviour expressed in verbal way
	/// \return
	/// \note SocialConductor-related
	std::string getBehaviourActive() const;

	virtual ~Move();

private:

	double dist_traveled_;

    /// \brief Based on actors parameters computes fuzzy
    /// output representing social behaviour
    fuzz::Processor fuzzy_processor_;

    /// \brief Social behaviour-based force generator
    fuzz::SocialConductor social_conductor_;

    /// \brief Helper class storing actor's path
    /// and distance to the closest obstacle
    std::shared_ptr<Path> path_storage_ptr_;

    /// \brief ParamLoader class acts as a local `ParameterServer` -
    /// stores parameters for a whole system (SFM too),
    /// avoids pollution of an Actor class with plenty of parameter
    /// variables
    std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr_;

    /// \brief Social Force Model interface object
	std::shared_ptr<sfm::SocialForceModel> sfm_ptr_;

    /// \brief Class that stores all the data needed by SFM that couldn't be saved in WorldPtr
    /// http://answers.gazebosim.org/question/22114/actor-related-information-in-gazebophysicsworldptr-and-collision-of-actors/
	std::shared_ptr<const actor::core::CommonInfo> common_info_ptr_;

    /// \brief Dynamic list of models to ignore. Used by SFM
    std::shared_ptr<const std::vector<std::string> > ignored_models_ptr_;

    /// \brief Pointer to the world, for convenience.
    gazebo::physics::WorldPtr world_ptr_;

    /// TODO:
    std::shared_ptr<const actor::core::Velocity> velocity_ptr_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_MOVE_H_ */
