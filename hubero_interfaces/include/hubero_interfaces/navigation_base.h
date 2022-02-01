#pragma once

#include <hubero_common/defines.h>
#include <hubero_common/logger.h>
#include <hubero_common/typedefs.h>
#include <string>

namespace hubero {

/**
 * @brief Creates interface class for algorithms that may provide navigation skills to the HuBeRo actors
 */
class NavigationBase {
public:
	/**
	 * @brief Default constructor
	 */
	NavigationBase(): initialized_(false), feedback_(TaskFeedbackType::TASK_FEEDBACK_UNDEFINED) {}

	/**
	 * @brief Initialization
	 */
	inline virtual bool initialize(const std::string& actor_name, const std::string& world_frame_name) {
		actor_name_ = actor_name;
		world_frame_name_ = world_frame_name;
		initialized_ = true;
	}

	/**
	 * @brief Evaluates whether given @ref goal is achievable starting from @ref start
	 *
	 * @note could be const, but ROS version uses non-const serviceclient::call inside
	 */
	inline virtual bool isPoseAchievable(const Pose3& start, const Pose3& goal, const std::string& frame) {
		return false;
	}

	/**
	 * @brief Related to localisation
	 */
	virtual void update(const Pose3& pose, const Vector3& vel_lin = Vector3(), const Vector3& vel_ang = Vector3()) {
		current_pose_ = pose;
	}

	/**
	 * @brief Related to simplest navigation task (moving to goal pose)
	 */
	virtual bool setGoal(const Pose3& pose, const std::string& frame) {
		goal_pose_ = pose;
		goal_frame_ = frame;
		return true;
	}

	/**
	 * @brief Tries to abort the movement goal, returns true if successful
	 */
	virtual bool cancelGoal() {
		return false;
	}

	/**
	 * @brief Returns TaskFeedbackType
	 */
	inline TaskFeedbackType getFeedback() const {
		return feedback_;
	}

	/**
	 * @brief Returns true if class was initialized successfully
	 */
	inline bool isInitialized() const {
		return initialized_;
	}

	/**
	 * @brief Returns newest velocity command
	 */
	inline virtual Vector3 getVelocityCmd() const {
		return Vector3();
	}

	/**
	 * @brief Retrieves newest goal pose
	 */
	inline virtual Pose3 getGoalPose() const {
		return goal_pose_;
	}

	/**
	 * @brief Retrieves newest goal's frame ID
	 */
	inline virtual std::string getGoalFrame() const {
		return goal_frame_;
	}

	/**
	 * @brief Retrieves name of the world (simulator) frame
	 */
	inline virtual std::string getWorldFrame() const {
		return world_frame_name_;
	}

protected:
	/// @brief Stores initialization indicator flag
	bool initialized_;

	/// @brief Name of the actor
	std::string actor_name_;

	/// @brief Navigation task feedback
	TaskFeedbackType feedback_;

	/// @brief Name of the frame that incoming ( @ref update ) poses are referenced in
	std::string world_frame_name_;

	/// @brief Stores most recent pose from localisation
	Pose3 current_pose_;

	/// @brief Stores newest goal pose
	Pose3 goal_pose_;

	/// @brief Stores newest goal pose frame
	std::string goal_frame_;
}; // class NavigationBase

} // namespace hubero
