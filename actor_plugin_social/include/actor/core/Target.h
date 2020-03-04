/*
 * Target.h
 *
 *  Created on: Jul 4, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ACTOR_CORE_TARGET_H_
#define INCLUDE_ACTOR_CORE_TARGET_H_

#include <actor/ros_interface/GlobalPlan.h>
#include <actor/ros_interface/ParamLoader.h>
#include <tuple>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>	// WorldPtr
#include <ignition/math/Vector3.hh>
#include <queue>
#include <gazebo/common/Time.hh>
#include <memory> // shared_ptr

#include <actor/inflation/Border.h>
#include <actor/core/CommonInfo.h>
#include <sfm/core/ActorInfoDecoder.h>

#include <actor/ros_interface/Conversion.h>
#include <actor/core/TargetLot.h>

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
	 * @brief Creates an internal copy of the CommonInfo instance
	 * which is used for triggering and maintaining the `follow object`
	 * state operation.
	 */
	void initializeCommonInfo(const std::shared_ptr<actor::core::CommonInfo> common_info_ptr);

	/**
	 * @brief Method to set new target for the actor. Tracked model is accessed via its name.
	 * This operation does abandon current target completely (it is not stored in the queue)
	 * but it does not change the target queue.
	 * If current state is object tracking already, then the tracked object will be overwritten.
	 * @param object_name: name of an object to follow
	 * @return True if object is valid and command has been accepted
	 */
	bool followObject(const std::string &object_name);

	/**
	 * @brief Evaluates target class' mode
	 * @return Boolean type flag indicating whether object tracking is active
	 */
	bool isFollowing() const;

	/**
	 * @brief Evaluates whether object tracking mode is active and if followed object
	 * has been reached (i.e. is close enough, within a `target_tolerance` parameter).
	 * @return False if object tracking mode is not active
	 */
	bool isFollowedObjectReached() const;

	/**
	 * @brief Tries to find `the closest` edge of the tracked object's bounding box,
	 * evaluates whether a global plan reaching that point can be found.
	 * @return True if object tracking is still possible, false if path to the object
	 * cannot be found.
	 * @note This method is blocked to perform operations too often. Once per few seconds
	 * should be sufficient.
	 */
	bool updateFollowedTarget();

	/**
	 * @brief Stops following the selected model, resets internal state so another command
	 * (follow, new target point) can be processed
	 * @return True if stopped successfully, false if following was not active
	 */
	bool stopFollowing();

	/**
	 * @brief Method to set a new target for the actor
	 * @param target - static object's position
	 * @param force_queue - whether to add the target straight to the queue (useful
	 * for adding targets to the queue at the startup - when global planner's
	 * costmap is not initialized yet)
	 * @return True if [x,y] position is valid
	 */
	bool setNewTarget(const ignition::math::Vector3d &target, bool force_queue = false);

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

	/* See `setNewTargetPriority()` template method at the bottom of the file */

    /**
     * @brief Tries to find some new (random) target location that is far enough from the current one
     * and is reachable in terms of global plan
     * @param info
     * @return True if found and saved, false otherwise
     */
	bool chooseNewTarget();

	/**
	 * @brief Wrapper method for setNewTarget (from the queue) and chooseNewTarget.
	 * Consists of some conditions and calls to one of the above methods. It may also
	 * return false when some error occurs.
	 * @return True if target has been changed (taken from the queue or the a new one found)
	 */
	bool changeTarget();

    /**
     * @brief
     * @param start
     * @param target_to_be
     * @return
     */
    bool generatePathPlan(const ignition::math::Vector3d &start, const ignition::math::Vector3d &target_to_be);

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
     * @brief Clears the components of the poses vector standing for the global plan checkpoints
     */
    void resetPath();

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
	bool isTargetStillReachable();

    /**
     * @brief Checks if the target is not reached for a certain amount of time
     * @param info - Gazebo Update info
     * @return True is threshold time has elapsed
     */
    bool isTargetNotReachedForTooLong() const;

    /**
     * @brief Calculates a distance to the current target and evaluates if whole global path
     * has been traveled (i.e. the last checkpoint has been reached)
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
     * @brief Reads and clears `has_new_path_` flag.
     * @return True if path has been generated since last `isPathNew` call
     * @note Non-const due to flag clearance
     */
    bool isPathNew();

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
	 * @brief Evaluates if a given model is a world model in a `combined` form.
	 * `isModelNegligible` cannot be used because the model name must be given explicitly.
	 */
	static bool isCombinedWorldModel(const std::string &object_name, const std::string &world_model_name);

	/**
	 * @brief Helper, static method that checks if any obstacle's bounding box does contain the investigated point
	 * @param bb - bounding box
	 * @param pt - point to be checked
	 * @return True if BB does contain the point
	 */
	static bool doesBoundingBoxContainPoint(const ignition::math::Box &bb, const ignition::math::Vector3d &pt);

	/**
	 * @brief Helper function that checks if model of a given name exists in the world
	 * @param object_name
	 * @return A tuple of bool and ModelPtr, bool set to true if model is valid and ModelPtr is set accordingly.
	 * If bool is false, then ModelPtr is NULL
	 * @note Cannot be static (world_ptr_); utilized by Actor (see @ref lieDown)
	 */
	std::tuple<bool, gazebo::physics::ModelPtr> isModelValid(const std::string &object_name);

	/**
	 * @brief Starting from the position given by `start`, let's find a safe point according to the costmap
	 * @param start: start position
	 * @param end: end position
	 * @param min_dist_to_end: minimum distance to the end position, i.e. how far the search should be performed
	 * @return A tuple with bool True only if a safe position was found considering the `min_dist_to_end` condition.
	 * @note Non-const due to `global_planner's` getCost call
	 */
	std::tuple<bool, ignition::math::Vector3d> findSafePositionAlongLine(const ignition::math::Vector3d &start, const ignition::math::Vector3d &end, const double &min_dist_to_end = 2.0);

	/**
	 * @brief Similar to @ref findSafePositionAlongLine, but does not use costmap and global planner -
	 * the search is based on bounding boxes of world objects
	 * @param start: start position
	 * @param end: end position
	 * @return A tuple with bool True only if a empty position was found
	 */
	std::tuple<bool, ignition::math::Vector3d> findEmptyPositionAlongLine(const ignition::math::Vector3d &start, const ignition::math::Vector3d &end) const;

	/**
	 * @brief Destructor
	 */
	virtual ~Target();

private:

	/// @brief TODO
	typedef TargetLot<ignition::math::Vector3d> TargetLotV3d;

	/**
	 * TODO:
	 * @param target
	 * @param force_queue
	 * @return
	 * @note Other versions of @ref setNewTarget overloaded method are public
	 */
	bool setNewTarget(TargetLotV3d &target, bool force_queue = false);

	/**
	 * @brief Tries to generate global path plan. If successful - updates internal state,
	 * target position and next checkpoint position.
	 * @param pt: prospective new target
	 * @return True if new target selected
	 */
	//bool tryToApplyTarget(const ignition::math::Vector3d &pt);
	bool tryToApplyTarget(const TargetLotV3d& target_lot, const ignition::math::Vector3d &start);

	bool tryToApplyTarget(const TargetLotV3d& target_lot);

	/**
	 * @brief Finds the closest (according to a straight line) point connecting actor's
	 * position with model's bounding
	 * @param model_ptr: a ModelPtr to an object whose edge is searched
	 * @return Tuple: bool (True if found), Vector3 (intersection point coordinates), Vector3 (line direction - slope)
	 * @note Similar to sfm::core::Inflator components
	 * @note Non-const because of getCost service call
	 */
	std::tuple<bool, ignition::math::Vector3d, ignition::math::Vector3d> findBorderPoint(const gazebo::physics::ModelPtr model_ptr);

	/**
	 * @brief Helper function which checks whether a given point (moved away along a certain
	 * direction) is reachable in terms of costmap. Costmap's inflation must be considered.
	 * @param pt_intersection is a corner point
	 * @param quarter is a direction along which point will be moved away
	 * @return point
	 * @note Cannot be `const` because of internal `GlobalPlan::getCost` call
	 */
	ignition::math::Vector3d findReachableDirectionPoint(const ignition::math::Vector3d &pt_intersection, const unsigned int &quarter);

	/// @brief Resets all time stamps, their value will be set up to the current time.
	void resetTimestamps();

	/// @brief Calls GetCost service for the surroundings of {pos_x, pos_y}
	int16_t getCostMean(const double &pos_x, const double &pos_y);


    /// @brief Start location of the current movement (not necessarily `valid` in terms of costmap)
	ignition::math::Vector3d start_;

    /// @brief Current target location
    // ignition::math::Vector3d target_;
	TargetLotV3d target_;

    /// @brief Queue of consecutive target locations
    // std::queue<ignition::math::Vector3d> target_queue_;
    std::queue<TargetLotV3d> target_queue_;

    /// @brief A certain point from path to target_ (generated by global
    /// planner; it was chosen as closest checkpoint (according to current
    /// position) to the actor's target location
    ignition::math::Vector3d target_checkpoint_;

    /// @brief Time of the last new target selection
	gazebo::common::Time time_last_target_selection_;

    /// @brief Time of the last reachability test.
	gazebo::common::Time time_last_reachability_;

//	/// @brief Time of the last checkpoint `abandonability` test.
//	gazebo::common::Time time_last_abandonability_;

    /// @brief Global plan provider class;
    /// New global plan can be requested via this object
    actor::ros_interface::GlobalPlan global_planner_;

    /// @brief Flag indicating that actor has valid target set (does not mean
    /// that a global plan was already generated or found).
    bool has_target_;

    /// @brief Flag indicating that actor has a global plan for the current
    /// target generated. This flag is set `false` in target setter methods.
	bool has_global_plan_; // when target is chosen, this flag will be set `false`

	/// @brief Flag indicating that a new global path has just been generated.
	bool has_new_path_;

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

	/// @brief Flag indicating that actor's current state is object tracking (following)
	bool is_following_;

	/// @brief Flag indicating that actor is close enough to the tracked object
	/// so it can be stopped until the model move away.
	bool is_followed_object_reached_;

    /// \@brief Pointer to the currently followed object
    gazebo::physics::ModelPtr followed_model_ptr_;

    /// @brief Time of the last generation of a global plan for object tracking mode.
	gazebo::common::Time time_last_follow_plan_;

	/// @brief Stores the actor-related info which cannot be updated
	/// via gazebo's WorldPtr
	std::shared_ptr<actor::core::CommonInfo> common_info_ptr_;

	/// @brief Decoder of information stored in the CommonInfo class (container)
	sfm::ActorInfoDecoder common_info_decoder_;

public:

	/// @brief Whether a position is safe according to the costmap
	static const int16_t COST_THRESHOLD;

	/**
	 * @brief Updates the current target even if the target queue
	 * is not empty.
	 * @param target:
	 * @note The target is a place within costmap bounds or a name of an object.
	 * @param queue_backup: if TRUE (default) the previous queue elements
	 * will be pushed back to the container (this operation needs to clear
	 * the whole queue).
	 * @return True if operation successful
	 */
	template <typename T>
	bool setNewTargetPriority(const T &target, const bool &queue_backup = true) {

		// make a backup of a current target/target queue (it will be restored
		// after reachment of the given goal (having priority))
		std::vector<ignition::math::Vector3d> targets_v;

		// abandon the current target
		if ( isTargetChosen() ) {
			abandonTarget();
		}

		// while -> queue size is not known;
		// set and immediately abandon all targets stored
		// in the queue
		while ( !isTargetQueueEmpty() ) {
			if ( queue_backup ) {
				targets_v.push_back(getTarget());
			}
			setNewTarget(); 	// pull out a target from the queue
			abandonTarget();	// abandon it
		}

		// try to set a new target (desired one)
		bool status = setNewTarget(target);

		// whether to restore old queue elements
		if ( queue_backup ) { 	// although there are no elements in the `targets_v`
								// vector - let's omit that section if no backup needed
			// restore old targets (put them straight into the queue)
			for ( size_t i = 0; i < targets_v.size(); i++ ) {
				setNewTarget(targets_v.at(i), true);
			}
		}

		return (status);

	}

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_ACTOR_CORE_TARGET_H_ */
