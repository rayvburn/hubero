/*
 * Actor.h
 *
 *  Created on: Apr 9, 2019
 *      Author: rayvburn
 */

#ifndef SRC_CORE_ACTOR_H_
#define SRC_CORE_ACTOR_H_

// C++ STL
#include <string>
#include <vector>
#include <array>
#include <tuple>

// Actor-related
#include "core/Enums.h"
#include "core/CommonInfo.h"
#include "core/FSM.h"
#include "ros_interface/Node.h"
#include "ros_interface/Stream.h"
#include "ros_interface/ConnectionFwd.h" // must be here due to circular dependency
#include "ros_interface/Connection.h"

// Social Force Model
#include "sfm/core/SocialForceModel.h"
#include "sfm/core/SFMDebug.h"

// Inflation types for SFM
#include "inflation/Box.h"
#include "inflation/Circle.h"
#include "inflation/Ellipse.h"

// Gazebo
#include <gazebo-8/gazebo/common/UpdateInfo.hh>
#include <gazebo-8/gazebo/physics/Model.hh>
#include <gazebo-8/gazebo/physics/Actor.hh>
#include <gazebo-8/gazebo/physics/World.hh>
#include <sdformat-5.3/sdf/Element.hh>

// ignition
#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// ROS interface
#include "ActorROSInterface.h"

// -------------------------

namespace actor {
namespace core {


/* References:
 * https://stackoverflow.com/questions/11711034/stdshared-ptr-of-this
 * https://en.cppreference.com/w/cpp/memory/enable_shared_from_this */

class Actor : public std::enable_shared_from_this<actor::core::Actor> {

public:

	/// \brief Default contructor
	Actor();

	/// \brief Passes World and Actor pointers for use within Actor controller class
	void initGazeboInterface(const gazebo::physics::ActorPtr &actor, const gazebo::physics::WorldPtr &world);

    /// \brief Method that initializes publishers, services etc.
    void initRosInterface();

	/// \brief Read bounding type parameter and set accordingly bounding model's parameters
	/// \brief Set circular shape as an inflator
	/// \param[in] radius length of a circle
	void initInflator(const double &circle_radius);

	/// \brief Set rectangular shape as an inflator (uses axis aligned bounding box)
	/// \param[in] Half length of a side along X axis
	/// \param[in] Half length of a side along Y axis
	/// \param[in] Half length of a side along Z axis (height)
	void initInflator(const double &box_x_half, const double &box_y_half, const double &box_z_half);

	/// \brief Set circular shape as an inflator
	/// \param[in] Length of a semi-major axis (a)
	/// \param[in] Length of a semi-minor axis (b)
	/// \param[in] Offset from center along the X axis (positive shifts
	///            ellipse's center in front of the Actor
	/// \param[in] Offset from center along the Y axis (positive shifts
	/// 		   ellipse's center towards Actor's right-hand side
	void initInflator(const double &semi_major, const double &semi_minor, const double &center_offset_x, const double &center_offset_y);

	/// TODO:
	void initSFM(const double &param);

	/// \brief Search for actor-related fields in the .world file to load parameters
	void readSDFParameters(const sdf::ElementPtr sdf);

	/// \brief Method to set new target for actor - static objet's pose
	void setNewTarget(const ignition::math::Pose3d &pose);

	/// \brief Method to set new target for actor - object's name
	/// \return True if object is valid
	bool setNewTarget(const std::string &object_name);

	/// \brief Method to set new target for actor - object's name
	/// \return True if object is valid
	// TODO: stop_after_arrival handling
	bool followObject(const std::string &object_name, const bool &stop_after_arrival);

	/// \brief Get velocity vector (linear x, linear y and angular `yaw`)
	std::array<double, 3> getVelocity() const;

	/// \brief Set actor's new stance type (first see which are allowed)
	/// \return True if stance is valid
	bool setStance(const actor::ActorStance &stance_type);

	/// \brief Set actor's new state (first see which are allowed)
	/// \return True if state is valid
	bool setState(const actor::ActorState &new_state);

	/// \brief Executes handler for currently set state
	void executeTransitionFunction(const gazebo::common::UpdateInfo &info);

    /// \brief Handlers for each state
    void stateHandlerAlignTarget	(const gazebo::common::UpdateInfo &info);
    void stateHandlerMoveAround		(const gazebo::common::UpdateInfo &info);
    void stateHandlerFollowObject	(const gazebo::common::UpdateInfo &info);
    void stateHandlerTeleoperation 	(const gazebo::common::UpdateInfo &info);

	virtual ~Actor();


private:

    /// \brief Helper function to choose a new target location
    void chooseNewTarget(const gazebo::common::UpdateInfo &info);

    /// \brief Helper function to check if target is still
    /// reachable (for example after addition of a new model
    /// a current target may not be reachable any more)
    bool isTargetStillReachable(const gazebo::common::UpdateInfo &info);

    /// \brief Helper function to check if target is not
    /// reached for a certain amount of time
    bool isTargetNotReachedForTooLong(const gazebo::common::UpdateInfo &info) const;

	/// \brief Helper method that checks if any obstacle's bounding box
	/// does contain the investigated point
	bool doesBoundingBoxContainPoint(const ignition::math::Box &bb, const ignition::math::Vector3d &pt) const;

    /// \brief Method's that tries to align the actor's yaw angle
    /// to make him face his target
    /// \return: true if actor faces the target close enough (10 deg tolerance)
    bool alignToTargetDirection(ignition::math::Vector3d *rpy);

    /// \brief Helper functions that consider the offsets of the actor's yaw and a roll
    /// angles depending on current stance
	void updateStanceOrientation();

    /// \brief Function invoked at the start of each OnUpdate event
    /// \return: dt - time delta
    double prepareForUpdate(const gazebo::common::UpdateInfo &info);

    /// \brief Function invoked at the end of each OnUpdate event
    void applyUpdate(const gazebo::common::UpdateInfo &info, const double &dist_traveled);

    /// \brief Helper function to calculate the actor's velocity as it could not be set
    /// in WorldPtr - this is just a workaround for a Gazebo/ActorPlugin bug
    void calculateVelocity(const double &dt);

    /// \brief Helper function to set proper state handler according to current state
    void updateTransitionFunctionPtr();

    /// \brief Helper function that converts a stance type to associated animation
    std::string convertStanceToAnimationName() const;

    /// \brief Helper function that check if model of given name exists in the world
    inline std::tuple<bool, gazebo::physics::ModelPtr> isModelValid(const std::string &object_name) const;

    /// \brief Pointer to the parent actor.
    gazebo::physics::ActorPtr actor_ptr_;

    /// \brief Pointer to the world, for convenience.
    gazebo::physics::WorldPtr world_ptr_;

    /// \brief Current target location
    ignition::math::Vector3d target_;

    /// \brief Distance tolerance allowed
    double target_tolerance_;

    /// \brief Time of the last new target selection
	gazebo::common::Time time_last_target_selection_;

    /// \brief Time of the last reachability test.
	gazebo::common::Time time_last_reachability_;

    /// \brief Time of the last update.
    gazebo::common::Time time_last_update_;

    /// \brief Simple finite state machine object
    actor::core::FSM fsm_;

    /// \brief Stores pointer to a handler of a currently executed state
    void(Actor::*trans_function_ptr)(const gazebo::common::UpdateInfo &);

    /// \brief Class that stores all the data needed by SFM that couldn't be saved in WorldPtr
    /// http://answers.gazebosim.org/question/22114/actor-related-information-in-gazebophysicsworldptr-and-collision-of-actors/
    actor::core::CommonInfo common_info_;

    /// \brief Stores type of an actor's bounding for SFM
    actor::ActorBoundingType bounding_type_;

    /// \brief Object that acts as a inflation around the actor - box-shaped
    /// used by SFM, chosen as a parameter
    actor::inflation::Box bounding_box_;

    /// \brief Object that acts as a inflation around the actor - circle-shaped
    /// used by SFM, chosen as a parameter
    actor::inflation::Circle bounding_circle_;

    /// \brief Object that acts as a inflation around the actor - circle-shaped
    /// used by SFM, chosen as a parameter
    actor::inflation::Ellipse bounding_ellipse_;

    /// \brief Actor's current stance
    actor::ActorStance stance_;

    /// \brief Actor's world pose - note: the coordinate system of an actor
    /// is rotated 90 deg CCW around world coordinate system's Z axis
    ignition::math::Pose3d pose_world_;

    /// \brief Previous actor_ptr_'s pose
    ignition::math::Pose3d pose_world_prev_;

    /// \brief Actual linear velocity_ of the actor
    ignition::math::Vector3d velocity_lin_;

    /// \brief Actual linear velocity_ of the actor
    ignition::math::Vector3d velocity_ang_;

    /// \brief An array that stores last 50 velocities -
    /// average velocity is calculated to provide smoother SFM operation
    std::array<ignition::math::Vector3d, 50> velocities_lin_to_avg_;

    /// \brief Object that should be followed by actor
    std::string object_to_follow_;

    /// \brief List of models to ignore. Used for SFM
    std::vector<std::string> ignored_models_;

    /// \brief Social Force Model interface object
    sfm::core::SocialForceModel sfm_;

    /// \brief Custom trajectory info
    gazebo::physics::TrajectoryInfoPtr trajectory_info_;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor_ptr_'s walking animation.
    double animation_factor_;

    /// \brief Node for ROS Interface
    actor::ros_interface::Node node_;

    /// \brief Stream class instance used for ROS Interface
    /// acts as an output buffer of the inter-agent
    /// communication channel
    actor::ros_interface::Stream stream_;

    /// \brief Connection class instance used for ROS Interface
    /// acts as an input buffer of the inter-agent
    /// communication channel
    std::shared_ptr<actor::ros_interface::Connection> connection_ptr_;

};

} /* namespace core */
} /* namespace actor_ptr_ */

#endif /* SRC_CORE_ACTOR_H_ */
