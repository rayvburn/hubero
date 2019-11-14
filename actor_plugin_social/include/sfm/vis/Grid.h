/*
 * Grid.h
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_VIS_GRID_H_
#define INCLUDE_SFM_VIS_GRID_H_

#include <sfm/vis/Arrow.h>
#include <sfm/vis/MarkerBase.h>
#include <vector>

// ROS libraries
#include <visualization_msgs/MarkerArray.h>


namespace sfm {
namespace vis {

/// \brief Class that manages creation of a vector field
/// visualization; generates a grid of points and calculates
/// a social force in each of them (treats each position as
/// if an actor would in fact be located there)
class Grid : public Arrow {

public:

	/// \brief Default constructor
	Grid();

	/// \brief Creates a grid (std::vector
	/// of an ignition::math::Vector3d instances)
	void createGrid(const float &x_start, const float &x_end, const float &y_start, const float &y_end, const float &resolution);

	/// \brief Add a given marker to locally stored
	/// MarkerArray; creates a copy of a marker to
	/// change an ID (necessary for MarkerArray)
	void addMarker(const visualization_msgs::Marker &marker);

	/// \brief Function used to determine whether
	/// to stop grid composition; checks if grid
	/// index is equal to grid size
	bool isWholeGridChecked() const;

	/// \brief Based on previously created grid this
	/// method returns a position of a next grid
	/// element using locally stored grid index;
	/// NOTE: by invoking it twice one will omit
	/// some grid points
	ignition::math::Vector3d getNextGridElement();

	/// \brief Sets grid index to 0; must be invoked after
	/// each grid composition finish (or just before
	/// its start)
	void resetGridIndex();

	/// \brief Returns a MarkerArray of an appropriate
	/// size based on grid's size and resolution
	visualization_msgs::MarkerArray getMarkerArray() const;

	/// \brief Default destructor
	virtual ~Grid();

private:

	/// \brief A vector of grid points
	std::vector<ignition::math::Vector3d> grid_;

	/// \brief Stores current grid point's index
	/// in a vector
	size_t grid_index_;

	/// \brief An array of Markers
	visualization_msgs::MarkerArray marker_array_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_SFM_VIS_GRID_H_ */
