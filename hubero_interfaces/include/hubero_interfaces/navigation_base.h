#pragma once

#include <hubero_common/defines.h>
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
	inline virtual bool initialize() {
		initialized_ = true;
	}

	/**
	 * @brief Evaluates whether given @ref goal is achievable starting from @ref start
	 *
	 * @note could be const, but ROS version uses non-const serviceclient::call inside
	 */
	inline virtual bool isPoseAchievable(const Pose3& start, const Pose3& goal, const std::string& frame = "") {
		return false;
	}

	/**
	 * @brief Related to localisation
	 */
	virtual void setPose(const Pose3& pose, const std::string& frame = "") {}

	/**
	 * @brief Related to simplest navigation task (moving to goal pose)
	 */
	virtual bool setGoal(const Pose3& pose, const std::string& frame = "") {}

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

protected:
	bool initialized_;
	TaskFeedbackType feedback_;
}; // class NavigationBase

} // namespace hubero
