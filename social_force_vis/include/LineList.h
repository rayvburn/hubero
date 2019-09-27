/*
 * LineList.h
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_LINELIST_H_
#define INCLUDE_LINELIST_H_

#include <vector>

// ignition libraries
#include <ignition/math/Pose3.hh>
#include <MarkerBase.h>

// ROS libraries
#include <visualization_msgs/MarkerArray.h>

namespace sfm {
namespace vis {

/// \brief LineList class extends Base's functionality
/// with a LineList management (may be developed further)
/// http://wiki.ros.org/rviz/DisplayTypes/Marker#Line_List_.28LINE_LIST.3D5.29
class LineList : public MarkerBase {

public:

	/// \brief Default constructor
	LineList();

	/// \brief Method which wraps createLineList()
	visualization_msgs::MarkerArray createArray(const std::vector<ignition::math::Pose3d> &poses);

	/// \brief Method that passes position components
	/// of a poses to a createLineList() method which
	/// takes ignition::math::Vector3d
	visualization_msgs::Marker create(const ignition::math::Pose3d &p1, const ignition::math::Pose3d &p2, const unsigned int &line_id) const;

	/// \brief Function that creates a line list;
	/// each line could be created out of 2 points
	visualization_msgs::Marker create(const ignition::math::Vector3d &p1, const ignition::math::Vector3d &p2, const unsigned int &line_id) const;

	/// \brief Default destructor
	virtual ~LineList();

protected:

	/// \brief Stores a max ID of a line
	/// which allows to delete old lines
	unsigned int line_id_max_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_LINELIST_H_ */
