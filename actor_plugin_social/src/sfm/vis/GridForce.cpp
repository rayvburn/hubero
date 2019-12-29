/*
 * GridForce.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: rayvburn
 */

#include <sfm/vis/GridForce.h>

namespace sfm {
namespace vis {

GridForce::GridForce() {
	Arrow::max_length_ = 1.0;
}

void GridForce::createGrid(const float &x_start, const float &x_end, const float &y_start, const float &y_end, const float &resolution) {

	Grid::createGrid(x_start, x_end, y_start, y_end, resolution);

	// resolution becomes the arrow length
	Arrow::max_length_ = resolution;

}

GridForce::~GridForce() {
}

} /* namespace vis */
} /* namespace sfm */
