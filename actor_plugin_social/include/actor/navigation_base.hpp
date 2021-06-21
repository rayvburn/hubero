#include <hubero_common/typedefs.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

class NavigationBase {
public:
    NavigationBase() = default;
    virtual bool isPoseAchievable(const Pose3& pose, const std::string& frame = "") const = 0;
    virtual void setPose(const Pose3& pose, const std::string& frame = "") = 0;
    virtual void update(const sensor_msgs::LaserScan& scan) = 0;
    virtual void update(const sensor_msgs::LaserScan& scan, const sensor_msgs::Image& img, const sensor_msgs::PointCloud2& rgbd_pcl) = 0;
    virtual bool setGoal(const Pose3& pose, const std::string& frame = "") = 0;
    virtual Vector3 getVelocityCmd() const = 0;
    virtual ~NavigationBase() = default;
};