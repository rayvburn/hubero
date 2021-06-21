#include "navigation_ros.hpp"

NavigationROS::NavigationROS(/* args */) {

}
bool NavigationROS::isPoseAchievable(const Pose3& pose, const std::string& frame) const {

}
void NavigationROS::setPose(const Pose3& pose, const std::string& frame) {

}
void NavigationROS::update(const sensor_msgs::LaserScan& scan) {

}
void NavigationROS::update(const sensor_msgs::LaserScan& scan, const sensor_msgs::Image& img, const sensor_msgs::PointCloud2& rgbd_pcl) {

}
bool NavigationROS::setGoal(const Pose3& pose, const std::string& frame) {

}
Vector3 NavigationROS::getVelocityCmd() const {

}