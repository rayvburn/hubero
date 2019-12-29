/*
 * Stream.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: rayvburn
 */

#include <actor/ros_interface/Stream.h>

namespace actor {
namespace ros_interface {

// ------------------------------------------------------------------- //

Stream::Stream() { }

// ------------------------------------------------------------------- //

void Stream::setNodeHandle(std::shared_ptr<::ros::NodeHandle> nh_ptr) {
	nh_ptr_ = nh_ptr;
}

// ------------------------------------------------------------------- //

void Stream::setNamespace(const std::string &ns) {
	namespace_ = ns;
}

// ------------------------------------------------------------------- //

std::tuple<bool, ros::Publisher> Stream::findPublisherInMap(const unsigned int &id) {

	std::map<unsigned int, ros::Publisher>::const_iterator it;
	it = publisher_id_map_.find(id);
	if ( it != publisher_id_map_.end() ) {
		return (std::make_tuple( true, it->second ));
	}
	return (std::make_tuple( false, ros::Publisher() ));

}

// ------------------------------------------------------------------- //

Stream::~Stream() { }

// ------------------------------------------------------------------- //

} /* namespace ros_interface */
} /* namespace actor */
