/*
 * Point.cpp
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#include <Point.h>

namespace sfm {
namespace vis {

// ------------------------------------------------------------------- //

Point::Point() { }

// ------------------------------------------------------------------- //

void Point::setColorLine(const float &r, const float &g, const float &b, const float &alpha) {
	setColor(&color_line_, r, g, b, alpha);
}

// ------------------------------------------------------------------- //

visualization_msgs::MarkerArray Point::createLineListArray(const std::vector<ignition::math::Pose3d> &poses) const {

	visualization_msgs::MarkerArray array;

	// check is there is even number of elements
	if ( poses.size() % 2 != 0 ) {
		std::cout << "createLineListArray() - odd number of elements in vector but MUST be even!" << std::endl;
		return (array);
	}

	for ( size_t i = 0; i < poses.size(); i=i+2 ) {
		array.markers.push_back( createLineList(poses.at(i), poses.at(i+1)) );
	}
	return (array);

}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Point::createLineList(const ignition::math::Pose3d &p1, const ignition::math::Pose3d &p2) const {
	return ( createLineList(p1.Pos(), p2.Pos()) );
}

// ------------------------------------------------------------------- //

visualization_msgs::Marker Point::createLineList(const ignition::math::Vector3d &p1, const ignition::math::Vector3d &p2) const {

	visualization_msgs::Marker marker;

	// NOTE: header.stamp, ns, id DEPRECATED here
	// `ADD is something of a misnomer, it really means "create or modify"`
	// http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/Marker.html

	marker.header.frame_id = frame_;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.orientation.w = 1.0;

	// line width
	marker.scale.x = 0.05;

	marker.color = color_line_;

	geometry_msgs::Point point;

	point.x = p1.X();
	point.y = p1.Y();
	point.z = 0.0f;

	marker.points.push_back(point);

	point.x = p2.X();
	point.y = p2.Y();
	point.z = 0.0f;

	marker.points.push_back(point);

	return (marker);

}

// ------------------------------------------------------------------- //

Point::~Point() { }

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace sfm */
