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
#include <actor/core/CommonInfo.h>
#include <actor/core/Enums.h>
#include <actor/core/FSM.h>
#include <actor/core/LieDownHelper.h>
#include <actor/core/Path.h>
#include <actor/core/Target.h>
#include <actor/inflation/Box.h>
#include <actor/inflation/Circle.h>
#include <actor/inflation/Ellipse.h>
#include <actor/ros_interface/Connection.h>
#include <actor/ros_interface/ConnectionFwd.h> // must be here due to circular dependency
#include <actor/ros_interface/GlobalPlan.h>
#include <actor/ros_interface/Node.h>
#include <actor/ros_interface/ParamLoader.h>
#include <actor/ros_interface/Stream.h>
#include <fuzz/Processor.h>
#include <fuzz/SocialConductor.h>

// Visualization
#include <sfm/vis/Arrow.h>
#include <sfm/vis/Grid.h>
#include <sfm/vis/LineList.h>
#include <sfm/vis/Text.h> // FIXME: path

// Gazebo
#include <gazebo-9/gazebo/common/UpdateInfo.hh>
#include <gazebo-9/gazebo/physics/Model.hh>
#include <gazebo-9/gazebo/physics/Actor.hh>
#include <gazebo-9/gazebo/physics/World.hh>
#include <sdformat-6.0/sdf/Element.hh>

// ignition
#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

// Extra
#include <sfm/core/SFMDebug.h>
#include <sfm/core/SocialForceModel.h>


// Processor - checking fuzzylite behavior
// TODO: change folder

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

	/* SCROLL DOWN for a `setNewTarget()` template */

	/// \brief Method to set new target for actor - object's name
	/// \note Object tracking will not run properly when a static
	/// obstacle (the one marked on the actor global costmap) is selected
	/// (whose name is given by `object_name`). Choose a dynamic obstacle
	/// or put an extra object in a free map cell and move it if wish to debug.
	/// \return True if object is valid
	bool followObject(const std::string &object_name);

	/// \brief Finishes `followObject` mode operation.
	bool followObjectStop();

	/// \brief Processes command related to the FSM's state switch to `LieDown`.
	/// \note Target point determined via object name.
	/// \param object_name: object on which actor should lie down
	/// \param height: height above the ground while lying
	/// \param rotation: additional rotation applied to the actor while lying
	/// \return True if operation successful
	bool lieDown(const std::string &object_name, const double &height, const double &rotation);

	/// \brief Processes command related to the FSM's state switch to `LieDown`.
	/// \note Target point determined via world coordinates
	/// \param x_pos
	/// \param y_pos
	/// \param z_pos: height above the ground while lying
	/// \param rotation: additional rotation applied to the actor while lying
	/// \return True if operation successful
	bool lieDown(const double &x_pos, const double &y_pos, const double &z_pos, const double &rotation);

	/// \brief Processes `LieDown` state finish command.
	/// \return
	bool lieDownStop();

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
    /// TODO: get rid of these UpdateInfos
    void stateHandlerAlignTarget	(const gazebo::common::UpdateInfo &info);
    void stateHandlerMoveAround		(const gazebo::common::UpdateInfo &info);
    void stateHandlerTargetReaching (const gazebo::common::UpdateInfo &info);
    void stateHandlerLieDown		(const gazebo::common::UpdateInfo &info);
    void stateHandlerStopAndStare	(const gazebo::common::UpdateInfo &info);
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
    bool manageTargetMovingAround();

    /// \brief Performs few target_manager's condition checks to decide whether
    /// an actor should stay in the object tracking mode.
    /// \return True if actor should still try to follow the currently selected object
    bool manageTargetTracking();

    /// \brief Performs few target_manager's condition checks to decide whether
	/// a current target is still reachable or the actor's FSM should change its
	/// state to `stop and stare`.
    bool manageTargetSingleReachment();

    /// \brief Calculates social force and new pose (based on SF). Computes
    /// traveled distance and updates world pose.
    /// \param dt: delta of time since last update event (in seconds)
    /// \return Traveled distance in meters
    double move(const double &dt);

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

    /// \brief Publishes position data as ROS TF messages.
    void visualizePositionData();

    /// \brief Social Force Model calculations visualization. Updates
    /// markers showing for example lengths of force vectors.
    /// Publishes visualization markers as ROS messages.
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

    // FIXME: bounding_ptr
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

    /// \brief Dynamic list of models to ignore. Used by SFM
    std::vector<std::string> ignored_models_;

    /// \brief Social Force Model interface object
    sfm::SocialForceModel sfm_;

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

	/// \brief Time of the last SFM-generated data publication
	gazebo::common::Time time_last_vis_sfm_pub_;

	/// \brief Time of the last TransformFrames message publication
	gazebo::common::Time time_last_tf_pub_;

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

    /// \brief Helper class to handle LIE_DOWN state
    LieDownHelper lie_down_;

    /// \brief Based on actors parameters computes fuzzy
    /// output representing social behaviour
    fuzz::Processor fuzzy_processor_;

    /// \brief Social behaviour-based force generator
    fuzz::SocialConductor social_conductor_;

    /// \brief Helper class storing actor's path
    /// and distance to the closest obstacle
    Path path_storage_{0.05}; // resolution

public:

    /// \brief Template method for setting a new target for actor (single reachment
    /// mode is supported only).
    /// \note The target is a place within costmap bounds or a name of an object.
    /// \param target: name of a target or its pose
    /// \return True if [x,y] position is valid and request accepted
	template <typename T>
	bool setNewTarget(const T &target) {

		bool status = target_manager_.setNewTargetPriority(target, true);

		// if new-desired target is achievable then change the state (align firstly)
		if ( status ) {
			setState(ActorState::ACTOR_STATE_TARGET_REACHING); // will be restored
			setState(ActorState::ACTOR_STATE_ALIGN_TARGET);
			setStance(ActorStance::ACTOR_STANCE_WALK);
		}

		return (status);

	}

};

} /* namespace core */
} /* namespace actor_ptr_ */

#endif /* SRC_CORE_ACTOR_H_ */