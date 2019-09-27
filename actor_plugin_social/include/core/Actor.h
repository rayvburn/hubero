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
#include <queue>

// Actor-related
#include "core/Enums.h"
#include "core/CommonInfo.h"
#include "core/FSM.h"
#include "core/Target.h"
#include "ros_interface/Node.h"
#include "ros_interface/Stream.h"
#include "ros_interface/ConnectionFwd.h" // must be here due to circular dependency
#include "ros_interface/Connection.h"
#include "ros_interface/ParamLoader.h"
#include "ros_interface/GlobalPlan.h"

// Social Force Model
#include "sfm/core/SocialForceModel.h"
#include <Arrow.h>
#include <Grid.h>
#include <LineList.h>
#include <Text.h> // FIXME: path
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
//#include "ActorROSInterface.h" // FIXME: old

// -------------------------

namespace actor {
namespace core {

/**
 * \brief Main controller (control subsystem) of the actor
 * \note Inheritance from std::enable_shared_from_this allows to pass
 * a std::shared_ptr of an Actor instance to Connection class which
 * provides ROS interface (`RX-only type`) and which must have
 * an ability to Actor's setter methods - thus a circular
 * dependency problem arises.
 *
 * Connection class stores weak_ptr (instead of a shared_ptr)
 * of an Actor instance - this prevents some memory leaks after
 * closing the application.
 *
 * \ref: https://en.cppreference.com/w/cpp/memory/enable_shared_from_this
 * \ref: https://stackoverflow.com/questions/11711034/stdshared-ptr-of-this
 */
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
	void initSFM();

	/// \brief Configure actor's initial pose, stance,
	/// FSM states, their transition functions, etc.
	/// based on provided parameters;
	/// must be called after initRosInterface() because uses
	/// GlobalPlan instance
	void initActor(const sdf::ElementPtr sdf);

	/// \brief Search for actor-related fields in the .world file to load parameters
	void readSDFParameters(const sdf::ElementPtr sdf);

	/// \brief Method to set new target for actor - static objet's pose
	/// \return True if [x,y] position is valid
	bool setNewTarget(const ignition::math::Pose3d &pose);

	/// \brief Method to set new target for actor - object's name
	/// \return True if object is valid
	bool setNewTarget(const std::string &object_name);

	/// \brief Method to set new target for actor - object's name
	/// \return True if object is valid
	bool followObject(const std::string &object_name, const bool &stop_after_arrival); 	// TODO: stop_after_arrival handling

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

    /// \brief Default destructor
	virtual ~Actor();

private:

    /// \brief Method's that tries to align the actor's yaw angle
    /// to make him face his target
    /// \return: true if actor faces the target close enough (10 deg tolerance)
    bool alignToTargetDirection(ignition::math::Vector3d *rpy);

    /// \brief Helper functions that consider the offsets of the actor's yaw and a roll
    /// angles depending on current stance
	void updateStanceOrientation(ignition::math::Pose3d &pose);

    /// \brief Function invoked at the start of each OnUpdate event
    /// \return: dt - time delta
    double prepareForUpdate();

    /// \brief Function invoked at the end of each OnUpdate event
    void applyUpdate(const double &dist_traveled);

    /// \brief Performs few target_manager's condition checks to decide whether
    /// a new target should be set or the current one should remain.
    /// \return True if actor should still try to reach the current target
    bool manageTarget();

    /// \brief Helper function to calculate the actor's velocity as it could not be set
    /// in WorldPtr - this is just a workaround for a Gazebo/ActorPlugin bug
    void calculateVelocity(const double &dt);

    /// \brief Helper function to set proper state handler according to current state
    void updateTransitionFunctionPtr();

    /// \brief Helper function to update a pose of a bounding model
    /// \param[in] Pose which model needs to be located in;
    /// parameter is useful for SFM grid visualization where
    /// actor's fake positions are passed to bounding model
    void updateBounding(const ignition::math::Pose3d &pose);

    /// \brief Helper function that converts a stance type to associated animation
    std::string convertStanceToAnimationName() const;

    // TODO
    void visualizePositionData();

    // TODO
    void visualizeSfmCalculations();

    /// \brief Helper function to create visualization_msgs::Marker(s)
    /// for a SFM's visualization
    /// \return True if MarkerArray is ready to be published
    bool visualizeVectorField();

    /// \brief Pointer to the parent actor.
    gazebo::physics::ActorPtr actor_ptr_;

    /// \brief Pointer to the world, for convenience.
    gazebo::physics::WorldPtr world_ptr_;

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
    std::shared_ptr<ignition::math::Pose3d> pose_world_ptr_;

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

    /// \brief List of models to ignore. Used by SFM
    std::vector<std::string> ignored_models_;

    /// \brief Social Force Model interface object
    sfm::core::SocialForceModel sfm_;

    /// \brief Social Force Model arrows visualization
	sfm::vis::Arrow sfm_vis_arrow_;

	/// \brief Social Force Model arrows visualization
	sfm::vis::LineList sfm_vis_line_list_;

    /// \brief Social Force Model grid visualization
	sfm::vis::Grid sfm_vis_grid_;

	/// \brief Actor active behaviour visualization based on SFM-generated data
	sfm::vis::Text sfm_vis_text_;

    /// \brief Time of the last force field visualization publication
	gazebo::common::Time time_last_vis_grid_pub_;

	/// \brief Time of the last force SFM-generated data publication
	gazebo::common::Time time_last_vis_sfm_pub_;

    /// \brief Custom trajectory info
    gazebo::physics::TrajectoryInfoPtr trajectory_info_;

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

    /// \brief ParamLoader class acts as a local `ParameterServer` -
    /// stores parameters for a whole system (SFM too),
    /// avoids pollution of an Actor class with plenty of parameter
    /// variables
    std::shared_ptr<actor::ros_interface::ParamLoader> params_ptr_;

    /// \brief Target class manages target-related operations,
    /// internally has a GlobalPlan class which is viable of
    /// calling ROS global_planner server asking for a global
    /// plan to be generated to a newly chosen target.
    Target target_manager_;

};

} /* namespace core */
} /* namespace actor_ptr_ */

#endif /* SRC_CORE_ACTOR_H_ */
