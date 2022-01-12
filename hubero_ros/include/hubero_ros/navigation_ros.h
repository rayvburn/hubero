#pragma once

#include <hubero_interfaces/navigation_base.h>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

namespace hubero {

/**
 * @brief Implements ROS interface for navigation tasks realization in HuBeRo
 *
 * @details @ref move_base is used as a planning interface
 * With GazeboRos plugins, we expect that perception data (i.e. LaserScan (and possibly Image with PCL))
 * are published to ROS topics. Topics configuration (for costmap generation) is placed inside YAML files.
 */
class NavigationRos: public NavigationBase {
public:
    /**
     * @brief Constructor
     */
    NavigationRos();

    /**
     * @brief Initializes internal components of the class
     *
     * @param agent_name name of the agent, used as namespace for topics and services
     * @return true If initialized properly
     * @return false If not initialized properly
     */
    virtual bool initialize(const std::string& agent_name) override;

    /**
     * @brief Evaluates possibility of pose reachability via trying to plan the full path to the @ref goal
     *
     * @param start
     * @param goal
     * @param frame
     * @return true
     * @return false
     */
    virtual bool isPoseAchievable(const Pose3& start, const Pose3& goal, const std::string& frame = "") override;

    /**
     * @brief Set the pose (localisation)
     *
     * @param pose
     * @param frame
     */
    virtual void setPose(const Pose3& pose, const std::string& frame = "") override;

    /**
     * @brief Set the goal
     *
     * @param pose
     * @param frame
     * @return true
     * @return false
     */
    virtual bool setGoal(const Pose3& pose, const std::string& frame = "") override;

    /**
     * @brief Cancels current goal (one goal is allowed at once)
     *
     * @return true
     * @return false
     */
    virtual bool cancelGoal() override;

    /**
     * @brief Get the velocity command
     *
     * @return Vector3
     */
    virtual Vector3 getVelocityCmd() const override;

    /**
     * @brief Destructor
     *
     */
    virtual ~NavigationRos() = default;

protected:
    /**
     * @brief Callback for velocity command retrieval
     *
     * @param msg
     */
    void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Publisher pub_mb_goal_;
    ros::Publisher pub_mb_cancel_;
    ros::Subscriber sub_cmd_vel_;
    ros::ServiceClient srv_mb_get_plan_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    Vector3 cmd_vel_;
    std::mutex mutex_callback_;

    std::string agent_name_;
    bool initialized_;
};

} // namespace hubero
