/*
 * debug_link_states.cpp
 *
 *  Created on: Dec 8, 2018
 *      Author: rayvburn
 */

// this file is just temporary to visualize what Gazebo published on link_states topic

// --------------------------------------------------------------
#define DEBUG_LINK_NOT_FOUND_FLAG 	999u
//#define HIPS_REFERENCE
// --------------------------------------------------------------

#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdint.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/LinkStates.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// --------------------------------------------------------------

static const std::string TF_PARENT_FRAME = "map";
static uint hips_idx_vector = 3;

static std::vector<std::string> link_to_debug_names;
static std::vector<std::string> link_to_debug_topic_names;

static std::vector<ros::Publisher> link_pose_pub;
static std::vector<ros::Publisher> link_twist_pub;

// for convenience
static std::vector<ros::Publisher> link_roll_pub;
static std::vector<ros::Publisher> link_pitch_pub;
static std::vector<ros::Publisher> link_yaw_pub;

static geometry_msgs::Pose pose;
static geometry_msgs::Twist twist;

// for convenience
static std_msgs::Float64 roll_msg;
static std_msgs::Float64 pitch_msg;
static std_msgs::Float64 yaw_msg;

static tf2_ros::TransformBroadcaster* broadcaster_ptr;
static geometry_msgs::TransformStamped tf_stamped;
// static geometry_msgs::TransformStamped last_hips_tf;	// in world coordinate frame / to avoid tf_listener

// --------------------------------------------------------------

// prototypes
static std::string	GetTopicFromLinkName(const std::string &link_name);
static uint 		GetVectorIdxWithLinkName(const std::string &link_name);
static void 		FillTwistPoseTF(const gazebo_msgs::LinkStates::ConstPtr _msg,
									const uint &_index,
									const std::string &_frame);

// --------------------------------------------------------------

void LinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg) {

	for ( uint i = 0; i < msg->name.size(); i++ ) {

		uint idx = GetVectorIdxWithLinkName(msg->name[i]);
		if ( idx == DEBUG_LINK_NOT_FOUND_FLAG ) continue;

		FillTwistPoseTF(msg, i, link_to_debug_topic_names[idx]);
		link_pose_pub[idx].publish(pose);
		link_twist_pub[idx].publish(twist);
		link_roll_pub[idx].publish(roll_msg);
		link_pitch_pub[idx].publish(pitch_msg);
		link_yaw_pub[idx].publish(yaw_msg);
		broadcaster_ptr->sendTransform(tf_stamped);

	}
}

// --------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "debug_link_states");
  ros::NodeHandle nh;

  /* Below is just a workaround for the issue connected with
   * tf_broadcaster which initializes some ROS functions by itself. It can't
   * be global in this case, because ros::init is invoked afterwards (but must be first) */
  tf2_ros::TransformBroadcaster broadcaster;
  broadcaster_ptr = &broadcaster;

  // new instances of actor(s)'s links can be added here
  link_to_debug_names.push_back("actor1::Head");
  link_to_debug_names.push_back("actor1::LeftShoulder");
  link_to_debug_names.push_back("actor1::Spine");
  link_to_debug_names.push_back("actor1::Hips");	
  link_to_debug_names.push_back("actor1::LHipJoint");
  link_to_debug_names.push_back("actor1::LowerBack");
  link_to_debug_names.push_back("actor1::LeftUpLeg");
  link_to_debug_names.push_back("actor1::LeftLeg");
  link_to_debug_names.push_back("actor1::LeftFoot");
  link_to_debug_names.push_back("actor1::LeftToeBase");


  // create publishers for all links
  for ( size_t i = 0; i < link_to_debug_names.size(); i++ ) {

	  ( link_to_debug_names[i] == "actor1::Hips" ) ? (hips_idx_vector = i) : (0); // reference frame
	  link_to_debug_topic_names.push_back(GetTopicFromLinkName(link_to_debug_names[i]));
	  // std::cout << i << " " << link_to_debug_topic_names[i] << std::endl;
	  link_pose_pub.push_back( nh.advertise<geometry_msgs::Pose> ("/gazebo/link_states/"+link_to_debug_topic_names[i]+"/pose",  3));
	  link_roll_pub.push_back( nh.advertise<std_msgs::Float64>   ("/gazebo/link_states/"+link_to_debug_topic_names[i]+"/orientation/r",  3));
	  link_pitch_pub.push_back(nh.advertise<std_msgs::Float64>   ("/gazebo/link_states/"+link_to_debug_topic_names[i]+"/orientation/p",  3));
	  link_yaw_pub.push_back(  nh.advertise<std_msgs::Float64>   ("/gazebo/link_states/"+link_to_debug_topic_names[i]+"/orientation/y",  3));
	  link_twist_pub.push_back(nh.advertise<geometry_msgs::Twist>("/gazebo/link_states/"+link_to_debug_topic_names[i]+"/twist", 3));

  }

  // create a subscriber of Gazebo messages
  ros::Subscriber link_states_sub =  nh.subscribe("/gazebo/link_states",  3, LinkStatesCallback);

  ros::spin();
  return 0;

}

// --------------------------------------------------------------

static std::string GetTopicFromLinkName(const std::string &link_name) {

	static uint counter = 0;
	std::string topic = link_name;
	size_t index = topic.find("::");

	if ( index != std::string::npos ) {
		std::replace( topic.begin(), topic.end(), ':', '_'); // replace all ':'s with '_'s
	} else {
		topic = "error_topic_nb" + counter;
		counter++;
	}

	return topic;

}

// --------------------------------------------------------------

static uint GetVectorIdxWithLinkName(const std::string &link_name) {

	for ( size_t i = 0; i < link_to_debug_names.size(); i++ ) {
		if ( link_name == link_to_debug_names[i] ) {
			return static_cast<uint>(i);
		}
	}
	return DEBUG_LINK_NOT_FOUND_FLAG;

}

// --------------------------------------------------------------

static void FillTwistPoseTF(const gazebo_msgs::LinkStates::ConstPtr _msg, const uint &_index, const std::string &_frame) {

	pose.position.x = _msg->pose[_index].position.x;
	pose.position.y = _msg->pose[_index].position.y;
	pose.position.z = _msg->pose[_index].position.z;

	pose.orientation.x = _msg->pose[_index].orientation.x;
	pose.orientation.y = _msg->pose[_index].orientation.y;
	pose.orientation.z = _msg->pose[_index].orientation.z;
	pose.orientation.w = _msg->pose[_index].orientation.w;

	twist.linear.x = _msg->twist[_index].linear.x;
	twist.linear.y = _msg->twist[_index].linear.y;
	twist.linear.z = _msg->twist[_index].linear.z;

	twist.angular.x = _msg->twist[_index].angular.x;
	twist.angular.y = _msg->twist[_index].angular.y;
	twist.angular.z = _msg->twist[_index].angular.z;

	tf_stamped.header.frame_id = TF_PARENT_FRAME;
	tf_stamped.child_frame_id = _frame;
	tf_stamped.transform.translation.x = _msg->pose[_index].position.x;
	tf_stamped.transform.translation.y = _msg->pose[_index].position.y;
	tf_stamped.transform.translation.z = _msg->pose[_index].position.z;

	tf_stamped.transform.rotation.x = _msg->pose[_index].orientation.x;
	tf_stamped.transform.rotation.y = _msg->pose[_index].orientation.y;
	tf_stamped.transform.rotation.z = _msg->pose[_index].orientation.z;
	tf_stamped.transform.rotation.w = _msg->pose[_index].orientation.w;


	// for R, P, Y extraction
	tf::Quaternion quat;
	quat.setX(_msg->pose[_index].orientation.x);
	quat.setY(_msg->pose[_index].orientation.y);
	quat.setZ(_msg->pose[_index].orientation.z);
	quat.setW(_msg->pose[_index].orientation.w);

	tf::Matrix3x3 rot(quat);
	// rot.getEulerYPR(yaw_msg.data, pitch_msg.data, roll_msg.data);
	rot.getRPY(roll_msg.data, pitch_msg.data, yaw_msg.data);	// same results

}
