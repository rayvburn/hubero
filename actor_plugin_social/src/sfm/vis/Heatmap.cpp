/*
 * Heatmap.cpp
 *
 *  Created on: Dec 27, 2019
 *      Author: rayvburn
 */

#include <sfm/vis/Heatmap.h>
#include <limits>

namespace sfm {
namespace vis {

Heatmap::Heatmap() {
	// TODO Auto-generated constructor stub

}

void Heatmap::scale(const double &max) {

	/*

	double maximum = std::numeric_limits<double>::min();

	// find the maximum value
	for (size_t i = 0; i < this->marker_array_.markers.size(); i++) {
		if ( this->marker_array_.markers.at(i) > maximum ) {
			maximum = this->marker_array_.markers.at(i);
		}
	}

	double multiplier;

	// scale all the markers
	for (size_t i = 0; i < this->marker_array_.markers.size(); i++) {
		this->marker_array_.markers.at(i) *= multiplier;
	}

	*/

}

Heatmap::~Heatmap() {
	// TODO Auto-generated destructor stub
}

} /* namespace vis */
} /* namespace sfm */
