/*
 * Base.cpp
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#include <sfm/vis/MarkerBase.h>



namespace sfm {
namespace vis {

// ------------------------------------------------------------------- //

MarkerBase::MarkerBase(): frame_("map") { }

// ------------------------------------------------------------------- //

void MarkerBase::init(const std::string &parent_frame) {
	frame_ = parent_frame;

	std::cout << "\n\n\n";
	std::cout << "[MarkerBase::init] actor::actor_global_frame_id/frame_: " << frame_ << std::endl;
	std::cout << "\n\n\n";

}

// ------------------------------------------------------------------- //

void MarkerBase::setColor(const float &r, const float &g, const float &b, const float &alpha) {
	setColor(&color_, r, g, b, alpha);
}

// ------------------------------------------------------------------- //


void MarkerBase::setColor(std_msgs::ColorRGBA *color, const float &r, const float &g, const float &b, const float &alpha) {
	color->a = alpha;	color->r = r;	color->g = g;	color->b = b;
}

// ------------------------------------------------------------------- //

MarkerBase::~MarkerBase() { }

// ------------------------------------------------------------------- //

} /* namespace vis */
} /* namespace sfm */
