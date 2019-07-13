/*
 * Stream.h
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_ROS_INTERFACE_STREAM_H_
#define INCLUDE_ROS_INTERFACE_STREAM_H_

// C++ STL
#include <memory> // std::shared_ptr
#include <map>
#include <tuple>

// ROS headers
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

// ignition library
#include <ignition/math/Pose3.hh>

// Gazebo
#include <gazebo/common/Time.hh>

// Actor-related
#include "core/Enums.h"

namespace actor {
namespace ros_interface {

class Stream {

public:

	/// \brief Default constructor
	Stream();

	/// \brief Loads a ros::NodeHandle pointer;
	/// A shared node is used for communication with ROS
	/// to avoid creating a separate node for each Actor
	void setNodeHandle(std::shared_ptr<ros::NodeHandle> nh_ptr);

	/// \brief Sets a namespace within all topics will be published;
	/// Usually namespace will be the actor's name
	void setNamespace(const std::string &ns);

	/* Scroll down for a templates definitions and overloaded methods */

	/// \brief Default destructor
	virtual ~Stream();

private:

	/// \brief Helper function that looks for a proper
	/// ros::Publisher based on an ID provided, where
	/// ID is an enum NUMBER defined in core/Enums.h
	std::tuple<bool, ros::Publisher> findPublisherInMap(const unsigned int &id);

	/// \brief Helper function for conversion from
	/// ignition's Pose to geometry_msgs' TransformStamped
	geometry_msgs::TransformStamped convertPoseToTfStamped(const ignition::math::Pose3d &pose) const;

	/// \brief NodeHandle's shared_ptr
	std::shared_ptr<ros::NodeHandle> nh_ptr_;

	/// \brief Namespace variable is a part of a topic name;
	/// used to distinguish messages sent by different Actor
	/// instances
	std::string namespace_;

	/// \brief A map that stores a ros::Publisher for each
	/// instance of an Actor's MarkerType, MarkerArrayType,
	/// TfType that was previously initialized
	std::map<unsigned int, ros::Publisher> publisher_id_map_;

	/// \brief A broadcaster to publish TF
	tf2_ros::TransformBroadcaster tf_broadcaster_;

public:

	/* NOTE:
	 * there seems to be a problem with template definition in a separate file -
	 * after successful compilation gzserver throws an error:
	 * gzserver: symbol lookup error: [PATH]/build/actor_plugin_social/libactor_core.so:
	 * undefined symbol: _ZN5actor13ros_interface6Stream11publishDataINS_15ActorMarkerType
	 * EN18visualization_msgs7Marker_ISaIvEEEEEvT_T0_
	 *
	 * when templates are defined in header everything seems to work */


	/// \brief Creates an object of ros::Publisher
	/// which broadcasts visualization_msgs::Marker
	/// or visualization_msgs::MarkerArray messages;
	/// NOTE: tf_broadcaster doesn't need initialization
	template <typename T1, typename T2>
	void initPublisher(const T1 &type, const std::string &topic_name) {

		ros::Publisher publisher;
		publisher = nh_ptr_->advertise<T2>(namespace_ + "/" + topic_name, 10);
		publisher_id_map_.insert( std::pair<unsigned int, ros::Publisher> (static_cast<unsigned int>(type), publisher) );

	}

	/// \brief Publishes a visualization_msgs::Marker
	/// or visualization_msgs::MarkerArray message (TODO: OR PATH?);
	/// message type is previously defined in ros::Publisher
	template <typename T1, typename T2>
	void publishData(const T1 &type, const T2 &marker_msg) {

		ros::Publisher pub;
		bool found = false;
		std::tie(found, pub) = findPublisherInMap( static_cast<unsigned int>(type) );

		// do not publish when nobody subscribes
		if ( (found) && (pub.getNumSubscribers() > 0) ) {
			pub.publish(marker_msg);
		}

	}

	/* below should be a specialization for template but it does
	 * not seem to be allowed to be done within class so a simple
	 * overloaded function is created */
	/// \brief Publishes a geometry_msgs::TransformStamped message;
	/// publisher initialization is not needed in this case (tf_broadcaster)
	void publishData(const ActorTfType &type, const ignition::math::Pose3d &pose
					 ,const gazebo::common::Time &sim_time
	){

		// append _goal to a namespace when broadcasting target pose
		std::string child_frame = namespace_;
		if ( type == ACTOR_TF_TARGET ) {
			child_frame.append("_goal");
		} else if ( type == ACTOR_TF_CHECKPOINT ) {
			child_frame.append("_checkpoint");
		}

		geometry_msgs::TransformStamped tf_stamp = convertPoseToTfStamped(pose);
		tf_stamp.header.frame_id = "world";
//		tf_stamp.header.stamp = ros::Time::now();
		tf_stamp.header.stamp.nsec = sim_time.nsec;
		tf_stamp.header.stamp.sec = sim_time.sec;
		tf_stamp.child_frame_id = child_frame;

		tf_broadcaster_.sendTransform(tf_stamp);

	}

	/// \brief Returns a number of subscribing units of
	/// a topic published by a ros::Publisher connected
	/// with a given `type`
	template <typename T>
	unsigned int getSubscribersNum(const T &type) {

		ros::Publisher pub;
		bool found = false;
		std::tie(found, pub) = findPublisherInMap( static_cast<unsigned int>(type) );

		if ( found ) {
			return ( pub.getNumSubscribers() );
		}

		// if not found then return 0
		return (0);

	}

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_STREAM_H_ */
