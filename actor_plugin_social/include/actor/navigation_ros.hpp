#include "navigation_base.hpp"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/GetPlan.h>

/**
 * @brief 
 * @details @ref move_base is used as a planning interface
 * With GazeboRos plugins, we expect that perception data (i.e. LaserScan (and possibly Image with PCL))
 * are published to ROS topics. Topics configuration (for costmap generation) is placed inside YAML files.
 */
class NavigationROS: public NavigationBase {
public:
    /**
     * @brief 
     * 
     */
    NavigationROS(/* args */);
    virtual bool isPoseAchievable(const Pose3& pose, const std::string& frame = "") const override;
    virtual void setPose(const Pose3& pose, const std::string& frame = "") override;
    virtual void update(const sensor_msgs::LaserScan& scan) override;
    virtual void update(const sensor_msgs::LaserScan& scan, const sensor_msgs::Image& img, const sensor_msgs::PointCloud2& rgbd_pcl) override;
    virtual bool setGoal(const Pose3& pose, const std::string& frame = "") override;
    virtual Vector3 getVelocityCmd() const override;
    virtual ~NavigationROS() = default;

protected:
    ros::NodeHandle nh_;
    ros::Publisher pub_mb_goal_;
    ros::Publisher pub_mb_cancel_;
    ros::ServiceClient srv_mb_get_plan_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};
