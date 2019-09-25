/*
 * Target.h
 *
 *  Created on: Jul 4, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_CORE_TARGET_H_
#define INCLUDE_CORE_TARGET_H_

#include <tuple>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>	// WorldPtr
#include <ignition/math/Vector3.hh>
#include <queue>
#include <gazebo/common/Time.hh>
#include <memory> // shared_ptr

#include "ros_interface/ParamLoader.h"
#include "ros_interface/GlobalPlan.h"

namespace actor {
namespace core {

/**
 * \brief Target manager
 */
class Target {

public:

	/**
	 * @brief Default constructor
	 */
	Target();

	/**
	 * @brief Parametrized constructor
	 * @param world_ptr
	 * @param pose_world_ptr
	 * @param params_ptr
	 */
	Target(gazebo::physics::WorldPtr world_ptr, std::shared_ptr<const ignition::math::Pose3d> pose_world_ptr,
		   std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr);

	/**
	 * @brief Copy constructor
	 * @param obj - object to be copied
	 */
	Target(const Target &obj);

	/**
	 * @brief Initializer of the `actor::ros_interface::GlobalPlan` class
	 * @param nh_ptr
	 * @param gap
	 * @param frame_id
	 */
	void initializeGlobalPlan(std::shared_ptr<ros::NodeHandle> nh_ptr, const size_t &gap, const std::string &frame_id);

	/**
	 * @brief Method to set new target for actor - object's name
	 * @param object_name
	 * @return True if object is valid
	 */
	bool followObject(const std::string &object_name); // FIXME?

	/**
	 * @brief Method to set a new target for the actor
	 * @param target - static object's position
	 * @return True if [x,y] position is valid
	 */
	bool setNewTarget(const ignition::math::Vector3d &target);

	/**
	 * @brief Method to set a new target for the actor
	 * @param target - static object's pose
	 * @return True if [x,y] position is valid
	 */
	bool setNewTarget(const ignition::math::Pose3d &pose);

	/**
	 * @brief Method to set a new target for the actor
	 * @param target - static object's name
	 * @return True if object is valid and free space could be found nearby
	 */
	bool setNewTarget(const std::string &object_name);

	/**
	 * @brief Sets new target as queue front element.
	 * @return True if operation successful
	 */
	bool setNewTarget();

    /**
     * @brief Tries to find some new (random) target location that is far enough from the current one
     * and is reachable in terms of global plan
     * @param info
     * @return True if found and saved, false otherwise
     */
	bool chooseNewTarget(); // const gazebo::common::UpdateInfo &info

	/**
	 * @brief Wrapper method for setNewTarget (from the queue) and chooseNewTarget.
	 * Consists of some conditions and calls to one of the above methods. It may also
	 * return false when some error occurs.
	 * @return True if target has been changed (taken from the queue or the a new one found)
	 */
	bool changeTarget();

    /**
     * @brief
     * @param target_to_be
     * @return
     */
    bool generatePathPlan(const ignition::math::Vector3d &target_to_be); // const ignition::math::Pose3d &pose_current,

    /**
     * @brief Discards the current target. Resets `has_target` flag.
     */
    void abandonTarget();

	/**
	 * @brief Sets checkpoint that is further away from the current one. All consecutive checkpoints
	 * are elements of Path vector (see GlobalPlan)
	 */
    void updateCheckpoint();

	/**
	 * @brief Checks whether costmap is initialized. Calls ROS Service Client.
	 * @return True if costmap is initialized
	 * @note Non-const because internally calls service. Does not change any part of the class though.
	 */
	bool isCostmapInitialized();

	/**
	 * @brief Checks whether global plan was generated for the current target
	 * @return True if plan is generated
	 */
	bool isPlanGenerated() const;

	/**
	 * @brief Checks `has_target` flag
	 * @return True if target is set
	 */
	bool isTargetChosen() const;

    /**
     * @brief Checks if the target is still reachable. For example after addition of a new model
     * the current target may not be reachable any more.
     * @param info - Gazebo Update info
     * @return
     */
	bool isTargetStillReachable(const gazebo::common::UpdateInfo &info);

    /**
     * @brief Checks if the target is not reached for a certain amount of time
     * @param info - Gazebo Update info
     * @return True is threshold time has elapsed
     */
    bool isTargetNotReachedForTooLong(const gazebo::common::UpdateInfo &info) const;

    /**
     * @brief Calculates distance to current target
     * @note Non-const because `has_target_` gets false`d when it turns out that target has been reached.
     * @return True if target is close enough. Threshold distance is a parameter value (`target_tolerance`).
     */
    bool isTargetReached();

    /**
     * @brief Checks if the queue containing consecutive target locations is empty.
     * @return True if empty
     */
    bool isTargetQueueEmpty() const;

    /**
     * @brief Calculates distance to currently chased checkpoint
     * @return True if checkpoint is close enough. Threshold distance is a parameter value (`target_tolerance`).
     */
    bool isCheckpointReached() const;

    /**
     * @brief Checks angle between actor's direction and checkpoint location.
     * @return If the angle is bigger than threshold then returns true (checkpoint
     * considered `abandonable` and a next one to be chosen).
     */
    bool isCheckpointAbandonable() const;

    /**
     * @brief Returns current checkpoint
     * @return Checkpoint
     */
    ignition::math::Vector3d getCheckpoint() const;

    /**
     * @brief Returns current target
     * @return Target
     */
	ignition::math::Vector3d getTarget() const;

	/**
	 * @brief Calls GlobalPlan method which creates a Path message based on vector of PoseStamped
	 * @return
	 */
	nav_msgs::Path getPath() const;

	/**
	 * @brief Helper, static method that checks if a given object is listed in a ignored_model vector passed in .YAML
	 * @param object_name
	 * @param dictionary
	 * @return True if model is listed in `ignored` list
	 * @note Static as it will be useful for SFM
	 */
	static bool isModelNegligible(const std::string &object_name, const std::vector<std::string> &dictionary);

	/**
	 * @brief Helper, static method that checks if any obstacle's bounding box does contain the investigated point
	 * @param bb - bounding box
	 * @param pt - point to be checked
	 * @return True if BB does contain the point
	 */
	static bool doesBoundingBoxContainPoint(const ignition::math::Box &bb, const ignition::math::Vector3d &pt);

	/**
	 * @brief Destructor
	 */
	virtual ~Target();

private:

	/**
	 * @brief Helper function that checks if model of a given name exists in the world
	 * @param object_name
	 * @return A tuple of bool and ModelPtr, bool set to true if model is valid and ModelPtr is set accordingly.
	 * If bool is false, then ModelPtr is NULL
	 */
	std::tuple<bool, gazebo::physics::ModelPtr> isModelValid(const std::string &object_name);

	/**
	 * @brief Helper function which checks whether a given point (moved away along a certain
	 * direction) is reachable in terms of costmap. Costmap's inflation must be considered.
	 * @param pt_intersection is a corner point
	 * @param quarter is a direction along which point will be moved away
	 * @return point
	 * @note Must not be const because of call to GlobalPlan::getCost
	 */
	ignition::math::Vector3d findReachableDirectionPoint(const ignition::math::Vector3d &pt_intersection, const unsigned int &quarter);

    /// @brief Current target location
    ignition::math::Vector3d target_;

    /// @brief Queue of consecutive target locations
    std::queue<ignition::math::Vector3d> target_queue_;

	// TODO: (from Actor.h) ?
	// queue of objects to follow (checkpoints)

    /// @brief A certain point from path to target_ (generated by global
    /// planner; it was chosen as closest checkpoint (according to current
    /// position) to the actor's target location
    ignition::math::Vector3d target_checkpoint_;

    /// @brief Time of the last new target selection
	gazebo::common::Time time_last_target_selection_;

    /// @brief Time of the last reachability test.
	gazebo::common::Time time_last_reachability_;

    /// @brief Global plan provider class;
    /// New global plan can be requested via this object
    actor::ros_interface::GlobalPlan global_planner_;

    /// @brief Flag indicating that actor has valid target set (does not mean
    /// that a global plan was already generated or found).
    bool has_target_;

    /// @brief Flag indicating that actor has a global plan for the current
    /// target generated. This flag is set `false` in target setter methods.
	bool has_global_plan_; // when target is chosen, this flag will be set `false`

	/// @brief Shared pointer to `pose_world` object from actor::core::Actor class.
	/// This is set in constructor.
	std::shared_ptr<const ignition::math::Pose3d> pose_world_ptr_;

	/// @brief Shared pointer to ParamLoader class. This is set in constructor.
	std::shared_ptr<const actor::ros_interface::ParamLoader> params_ptr_;

	/// @brief Pointer do world instance. This is set in constructor.
	/// @note WorldPtr is typedef'ed boost::shared_ptr. It is not marked
	/// const here but is read only in this (Target) class.
	/// This is set in constructor.
	gazebo::physics::WorldPtr world_ptr_;

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_CORE_TARGET_H_ */
