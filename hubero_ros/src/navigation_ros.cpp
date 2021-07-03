#include <hubero_ros/navigation_ros.h>

#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <hubero_common/converter.h>

#include <pluginlib/class_list_macros.h>
//register this class as a `hubero::interface::NavigationBase` plugin
PLUGINLIB_EXPORT_CLASS(hubero::NavigationROS, hubero::interface::NavigationBase)

namespace hubero {

NavigationROS::NavigationROS():
    nh_("hubero_navigation_ros"),
    agent_name_("actor"),
    initialized_(false)
{}

bool NavigationROS::initialize(const std::string& agent_name) {
    if (initialized_) {
        ROS_WARN("NavigationROS plugin was already initialized, aborting");
        return false;
    }

    ROS_INFO("NavigationROS::intialize: pub_mb_goal topic: %s, pub_mb_cancel topic: %s, srv_mb_get_plan name: %s, sub_cmd_vel topic: %s",
        ("/hubero/" + agent_name_ + "/move_base/navigation_ros/goal").c_str(),
        ("/hubero/" + agent_name_ + "/move_base/navigation_ros/cancel").c_str(),
        ("/hubero/" + agent_name_ + "/move_base/navigation_ros/get_plan").c_str(),
        ("/hubero/" + agent_name_ + "/move_base/cmd_vel").c_str()
    );
    agent_name_ = agent_name;
    // TODO: topic and srv names as parameters
    pub_mb_goal_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>(
        "/hubero/" + agent_name_ + "/move_base/navigation_ros/goal",
        3
    );
    pub_mb_cancel_ = nh_.advertise<actionlib_msgs::GoalID>(
        "/hubero/" + agent_name_ + "/move_base/navigation_ros/cancel",
        3
    );
    srv_mb_get_plan_ = nh_.serviceClient<nav_msgs::GetPlan>(
        "/hubero/" + agent_name_ + "/move_base/navigation_ros/get_plan"
    );
    sub_cmd_vel_ = nh_.subscribe(
        "/hubero/" + agent_name_ + "/move_base/cmd_vel",
        10,
        &NavigationROS::callbackCmdVel,
        this
    );
    initialized_ = true;
    return initialized_;
}

bool NavigationROS::isPoseAchievable(const Pose3& start, const Pose3& goal, const std::string& frame) {
    if (!initialized_) {
        ROS_WARN("NavigationROS plugin is not initialized, call `initialize` first");
        return false;
    }

    nav_msgs::GetPlan::Request req;
    nav_msgs::GetPlan::Response resp;
    req.start = converter::ignPoseToPoseStamped(start, frame);;
    req.goal = converter::ignPoseToPoseStamped(goal, frame);
    // TODO: param
    req.tolerance = 1.0f;
    bool success = srv_mb_get_plan_.call(req, resp);
    if (!success || resp.plan.poses.size() == 0) {
        return false;
    }
    // // TODO: verify
    return true;
}

void NavigationROS::setPose(const Pose3& pose, const std::string& frame) {
    if (!initialized_) {
        ROS_WARN("NavigationROS plugin is not initialized, call `initialize` first");
        return;
    }

    // shared between all published transforms
    auto stamp = ros::Time::now();

    std::string agent_base_frame_id = agent_name_ + "/base_footprint";

    auto eul = pose.Rot().Euler();
    printf("NavigationROS::setPose(): (1) x %1.2f, y %1.2f, z %1.2f, R %1.2f, P %1.2f, Y %1.2f\r\n",
            pose.Pos().X(),
            pose.Pos().Y(),
            pose.Pos().Z(),
            eul.X(),
            eul.Y(),
            eul.Z()
    );
    printf("Eul: R %1.2f, P %1.2f, Y %1.2f / Rot: R %1.2f, P %1.2f, Y %1.2f\r\n",
        eul.X(),
        eul.Y(),
        eul.Z(),
        pose.Rot().Roll(),
        pose.Rot().Pitch(),
        pose.Rot().Yaw()
    );

    geometry_msgs::TransformStamped transform_actor = converter::ignPoseToTfStamped(pose, false);
    transform_actor.header.frame_id = frame;
    transform_actor.header.stamp = stamp;
    transform_actor.child_frame_id = agent_base_frame_id;

    auto quat_actor = Quaternion(transform_actor.transform.rotation.w,
        transform_actor.transform.rotation.x,
        transform_actor.transform.rotation.y,
        transform_actor.transform.rotation.z
    );
    auto eul_actor_from_quat = quat_actor.Euler();
    printf("NavigationROS::setPose(): (2) x %1.2f, y %1.2f, z %1.2f, R %1.2f, P %1.2f, Y %1.2f\r\n",
        transform_actor.transform.translation.x,
        transform_actor.transform.translation.y,
        transform_actor.transform.translation.z,
        eul_actor_from_quat.X(),
        eul_actor_from_quat.Y(),
        eul_actor_from_quat.Z()
    );

    auto quat_actor_from_euler = Quaternion::EulerToQuaternion(eul_actor_from_quat);
    auto euler_test = quat_actor_from_euler.Euler();
    printf("NavigationROS::setPose(): (3) x %1.2f, y %1.2f, z %1.2f, R %1.2f, P %1.2f, Y %1.2f\r\n",
        transform_actor.transform.translation.x,
        transform_actor.transform.translation.y,
        transform_actor.transform.translation.z,
        euler_test.X(),
        euler_test.Y(),
        euler_test.Z()
    );

    tf_broadcaster_.sendTransform(transform_actor);
    transform_actor.child_frame_id = agent_name_ + "/map";
    tf_broadcaster_.sendTransform(transform_actor);
    transform_actor.child_frame_id = agent_name_ + "/odom";
    tf_broadcaster_.sendTransform(transform_actor);

    // transform from actor link to laser link
    geometry_msgs::TransformStamped transform_laser;
    transform_laser.header.frame_id = agent_base_frame_id;
    transform_laser.header.stamp = stamp;
    transform_laser.child_frame_id = agent_name_ + "/base_laser_link";
    transform_laser.transform.translation.x = 0.55;
    transform_laser.transform.translation.z = -0.8;
    transform_laser.transform.rotation.w = 1.0; // valid quaternion
    tf_broadcaster_.sendTransform(transform_laser);

    // transform from actor link to camera link
    geometry_msgs::TransformStamped transform_camera;
    transform_camera.header.frame_id = agent_base_frame_id;
    transform_camera.header.stamp = stamp;
    transform_camera.child_frame_id = agent_name_ + "/base_camera_link";
    // data reproduced from actor plugin declaration in a `world` file
    transform_camera.transform.translation.x = 0.55;
    transform_camera.transform.translation.z = 0.15;
    transform_camera.transform.rotation.w = 1.0; // valid quaternion
    tf_broadcaster_.sendTransform(transform_camera);
}

void NavigationROS::update(const sensor_msgs::LaserScan& scan) {
    if (!initialized_) {
        ROS_WARN("NavigationROS plugin is not initialized, call `initialize` first");
        return;
    }

    ROS_INFO("HuBeRo.NavigationROS::update: 1 (empty)");
}

void NavigationROS::update(
    const sensor_msgs::LaserScan& scan,
    const sensor_msgs::Image& img,
    const sensor_msgs::PointCloud2& rgbd_pcl
) {
    if (!initialized_) {
        ROS_WARN("NavigationROS plugin is not initialized, call `initialize` first");
        return;
    }

    ROS_INFO("HuBeRo.NavigationROS::update: 2 (empty)");
}

bool NavigationROS::setGoal(const Pose3& pose, const std::string& frame) {
    if (!initialized_) {
        ROS_WARN("NavigationROS plugin is not initialized, call `initialize` first");
        return false;
    }

    move_base_msgs::MoveBaseActionGoal goal;
    actionlib_msgs::GoalID goal_id;
    goal_id.id = "";
    goal_id.stamp = ros::Time::now();
    goal.goal_id = goal_id;
    goal.header.frame_id = frame;
    goal.goal.target_pose = converter::ignPoseToPoseStamped(pose, frame);
    pub_mb_goal_.publish(goal);
    ROS_INFO("HuBeRo.NavigationROS::setGoal: trying to set goal");
    return true;
}

bool NavigationROS::cancelGoal() {
    if (!initialized_) {
        ROS_WARN("NavigationROS plugin is not initialized, call `initialize` first");
        return false;
    }

    actionlib_msgs::GoalID goal_cancel;
    pub_mb_cancel_.publish(goal_cancel);
    ROS_INFO("HuBeRo.NavigationROS::cancelGoal: trying to cancel goal without specyfing stamp or ID!");
    return true;
}

Vector3 NavigationROS::getVelocityCmd() const {
    if (!initialized_) {
        ROS_WARN("NavigationROS plugin is not initialized, call `initialize` first");
        return Vector3();
    }
    return cmd_vel_;
}

void NavigationROS::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    const std::lock_guard<std::mutex> lock(mutex_callback_);
    cmd_vel_.X(msg->linear.x);
    cmd_vel_.Y(msg->linear.y);
    cmd_vel_.Z(msg->angular.z);
}

} // namespace hubero