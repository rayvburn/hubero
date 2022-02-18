#pragma once

#include <hubero_interfaces/status_base.h>

#include <ros/ros.h>
#include <hubero_ros/node.h>

namespace hubero {

/**
 * @brief Implements ROS interface to broadcast HuBeRo Actor's status
 */
class StatusRos: public StatusBase {
public:
    // TODO: would look cleaner with C++17 'static constexpr'
	/// Defines default size of the queue used for publisher
	static const int PUBLISHER_QUEUE_SIZE;

    /// How often publish updated status
    static const ros::Duration UPDATE_PUBLISH_PERIOD;

    StatusRos();

    virtual void initialize(
        std::shared_ptr<Node> node_ptr,
        const std::string& actor_name,
        const std::string& frame_name
    );

    /**
     * @brief Takes actor data update and broadcasts it to ROS topics
     */
    virtual void update(const Pose3& pose, const Vector3& vel_lin, const Vector3& vel_ang) override;

protected:
    /// ROS publisher to broadcast Actor status
    ros::Publisher pub_status_;

    /// Timestamp of last status broadcast (prevents too frequent publishes that are unnecessary)
    ros::Time time_last_pub_;

    /// Handy for counting actors that use this interface
    static int actor_num_;

    /// ID of the actor that uses this class
    int actor_id_;

}; // StatusRos

} // namespace hubero
