#include <hubero_ros/status_ros.h>
#include <hubero_common/logger.h>
#include <hubero_ros_msgs/Person.h>
#include <hubero_ros/utils/converter.h>
#include <hubero_ros/utils/misc.h>

namespace hubero {

const int StatusRos::PUBLISHER_QUEUE_SIZE = 15;
const ros::Duration StatusRos::UPDATE_PUBLISH_PERIOD = ros::Duration(0.02);
int StatusRos::actor_num_ = 0;

StatusRos::StatusRos(): StatusBase::StatusBase(), actor_id_(StatusRos::actor_num_) {
    // increase counter for other instances
    StatusRos::actor_num_++;
}

void StatusRos::initialize(
    std::shared_ptr<Node> node_ptr,
    const std::string& actor_name,
    const std::string& frame_name
) {
    if (this->isInitialized()) {
		HUBERO_LOG("[%s].[StatusRos] Already initialized, aborting\r\n", actor_name.c_str());
		return;
	}

    // initialize base class
    StatusBase::initialize(actor_name, frame_name);

    // helps loading params (looks from a global scope)
	ros::NodeHandle nh;

    // store name of the status topic
    std::string topic_status;
    std::string topic_status_param;
	nh.searchParam("/hubero_ros/" + actor_name + "/status_topic", topic_status_param);
	nh.param(topic_status_param, topic_status, std::string("actor/status"));

    pub_status_ = node_ptr->getNodeHandlePtr()->advertise<hubero_ros_msgs::Person>(
		topic_status,
		PUBLISHER_QUEUE_SIZE
	);

    HUBERO_LOG(
        "[%s].[StatusRos] Initialized ROS status publisher at '%s'\r\n",
        actor_name_.c_str(),
        pub_status_.getTopic().c_str()
    );
}

void StatusRos::update(const Pose3& pose, const Vector3& vel_lin, const Vector3& vel_ang) {
    if (!isInitialized()) {
		HUBERO_LOG("[%s].[StatusRos] Not initialized, call `initialize` first\r\n", actor_name_.c_str());
		return;
	}

    ros::Time time_current = ros::Time::now();
    if ((time_current - time_last_pub_) < StatusRos::UPDATE_PUBLISH_PERIOD) {
        // no need to publish that often
        return;
    }

    hubero_ros_msgs::Person person;
    // header
    person.header.frame_id = frame_pose_;
    person.header.stamp = time_current;
    // id of detection (simulated with actor ID)
    person.detection_id = actor_id_;
    // detector name
    person.name = "hubero";
    // name
    person.object_id = actor_name_;
    // pose
    person.pose.pose = ignPoseToMsgPose(pose);
    setIdealCovariance(person.pose.covariance);
    // velocity
    person.velocity.position = ignVectorToMsgPoint(vel_lin);
    person.velocity.orientation = ignVectorRpyToMsgQuaternion(vel_ang);
    // fully confident of pose and velocity
    person.reliability = 1.0;

    // additional fields - currently undefined

    pub_status_.publish(person);
    // save timestamp
    time_last_pub_ = time_current;
}

} // namespace hubero
