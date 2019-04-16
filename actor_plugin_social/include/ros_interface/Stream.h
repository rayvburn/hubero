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
//#include <nav_msgs/Path.h>

// ignition library
#include <ignition/math/Pose3.hh>

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

//	/// \brief Creates an object of ros::Publisher
//	/// which broadcasts visualization_msgs::Marker messages;
//	/// NOTE: tf_broadcaster doesn't need initialization
//	void initPublisher(const ActorMarkerType &type, const std::string &topic_name);
//
//	/// \brief Creates an object of a ros::Publisher type
//	/// which broadcasts visualization_msgs::MarkerArray messages
//	/// NOTE: tf_broadcaster doesn't need initialization
//	void initPublisher(const ActorMarkerArrayType &type, const std::string &topic_name);

	/// \brief Creates an object of a ros::Publisher type
	/// which broadcasts nav_msgs::Path messages
	/// NOTE: tf_broadcaster doesn't need initialization
	// TODO INCLUDE
//	void initPublisher(const ActorNavMsgType &type, const std::string &topic_name);


	/* Again - after successful compilation gzserver throws an error:
	 * gzserver: symbol lookup error: [PATH]/build/actor_plugin_social/libactor_core.so:
	 * undefined symbol: _ZN5actor13ros_interface6Stream11publishDataINS_15ActorMarkerType
	 * EN18visualization_msgs7Marker_ISaIvEEEEEvT_T0_
	/// \brief Publishes a Marker or MarkerArray with
	/// a proper publisher (which must be previously initialized)
	template <typename T1, typename T2>
	void publishData(const T1 type, const T2 marker_msg);
	*/

//	/// \brief Publishes visualization_msgs::Marker message
//	void publishData(const ActorMarkerType &type, const visualization_msgs::Marker &marker);
//
//	/// \brief Publishes visualization_msgs::MarkerArray message
//	void publishData(const ActorMarkerArrayType &type, const visualization_msgs::MarkerArray &marker);
//
	/// \brief Publishes a TF message
	void publishData(const ActorTfType &type, const ignition::math::Pose3d &pose);

	// TODO:
	// path

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

	template <typename T1, typename T2>
	void initPublisher(const T1 &type, const std::string &topic_name) {

		ros::Publisher publisher;
		publisher = nh_ptr_->advertise<T2>(namespace_ + topic_name, 10);
		publisher_id_map_.insert( std::pair<unsigned int, ros::Publisher> (static_cast<unsigned int>(type), publisher) );

	}

	template <typename T1, typename T2>
	void publishData(const T1 type, const T2 marker_msg) {

		ros::Publisher pub;
		bool found = false;
		std::tie(found, pub) = findPublisherInMap( static_cast<unsigned int>(type) );

		if ( found ) {
			pub.publish(marker_msg);
		}

	}

};

} /* namespace ros_interface */
} /* namespace actor */

#endif /* INCLUDE_ROS_INTERFACE_STREAM_H_ */
