/*
 * Cell.cpp
 *
 *  Created on: Dec 27, 2019
 *      Author: rayvburn
 */

#include <ignition/math/Quaternion.hh>
#include <sfm/vis/HeatCell.h>
#include <std_msgs/ColorRGBA.h>

namespace sfm {
namespace vis {

// --------------------------------------------------------------

HeatCell::HeatCell(): max_force_magnitude_(2000.0), min_force_magnitude_(250.0), resolution_(0.03) {}

// --------------------------------------------------------------

void HeatCell::setParameters(const double &min_force_magnitude, const double &max_force_magnitude, const double &resolution) {

	// range extended artificially (just in case of abnormal values)
	min_force_magnitude_ = 0.8 * min_force_magnitude;
	max_force_magnitude_ = 1.2 * max_force_magnitude;
	resolution_ = resolution;

}

// --------------------------------------------------------------

visualization_msgs::Marker HeatCell::create(const ignition::math::Vector3d &pos, const double &force_magnitude) const {

	visualization_msgs::Marker marker;
	// NOTE: header.stamp, ns, deprecated here
	// NOTE: marker.id is necessary for MarkerArray (otherwise only 1 marker will be drawn)
	// `ADD is something of a misnomer, it really means "create or modify"`
	// http://docs.ros.org/kinetic/api/visualization_msgs/html/msg/Marker.html

	marker.header.frame_id = this->frame_;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	// assign marker coordinates according to current point that is pointed by grid index
	marker.pose.position.x = pos.X();
	marker.pose.position.y = pos.Y();
	marker.pose.position.z = pos.Z() + 0.1; // few cm above the `ground`

	// default quaternion
	ignition::math::Quaterniond quaternion(0.0, 0.0, 0.0);

	marker.pose.orientation.x = quaternion.X();
	marker.pose.orientation.y = quaternion.Y();
	marker.pose.orientation.z = quaternion.Z();
	marker.pose.orientation.w = quaternion.W();

	// scale
	// arrow's length is calculated based on max allowable force `in SFM class`
	marker.scale.x = resolution_;
	marker.scale.y = marker.scale.x;
	marker.scale.z = 0.01;

	// convert force magnitude to HSV and then to RGB
	RgbColor rgb = HsvToRgb(convertMagnitudeToHSV(force_magnitude));

	// update marker data
	marker.color.a = 0.9;

	marker.color.r = rgb.r;
	marker.color.g = rgb.g;
	marker.color.b = rgb.b;

	return (marker);

}

// --------------------------------------------------------------

HeatCell::~HeatCell() {
	// TODO Auto-generated destructor stub
}

// --------------------------------------------------------------

HeatCell::RgbColor HeatCell::HsvToRgb(const HsvColor &hsv) const {

	RgbColor rgb;
	unsigned char region, remainder, p, q, t;

	if (hsv.s == 0)
	{
		rgb.r = hsv.v;
		rgb.g = hsv.v;
		rgb.b = hsv.v;
		return rgb;
	}

	region = hsv.h / 43;
	remainder = (hsv.h - (region * 43)) * 6;

	p = (hsv.v * (255 - hsv.s)) >> 8;
	q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
	t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

	switch (region)
	{
		case 0:
			rgb.r = hsv.v; rgb.g = t; rgb.b = p;
			break;
		case 1:
			rgb.r = q; rgb.g = hsv.v; rgb.b = p;
			break;
		case 2:
			rgb.r = p; rgb.g = hsv.v; rgb.b = t;
			break;
		case 3:
			rgb.r = p; rgb.g = q; rgb.b = hsv.v;
			break;
		case 4:
			rgb.r = t; rgb.g = p; rgb.b = hsv.v;
			break;
		default:
			rgb.r = hsv.v; rgb.g = p; rgb.b = q;
			break;
	}

	return rgb;

}

// --------------------------------------------------------------

HeatCell::HsvColor HeatCell::RgbToHsv(const RgbColor &rgb) const {

	HsvColor hsv;
	unsigned char rgbMin, rgbMax;

	rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
	rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

	hsv.v = rgbMax;
	if (hsv.v == 0)
	{
		hsv.h = 0;
		hsv.s = 0;
		return hsv;
	}

	hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
	if (hsv.s == 0)
	{
		hsv.h = 0;
		return hsv;
	}

	if (rgbMax == rgb.r)
		hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
	else if (rgbMax == rgb.g)
		hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
	else
		hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

	return hsv;

}

// --------------------------------------------------------------

HeatCell::HsvColor HeatCell::convertMagnitudeToHSV(const double &magnitude) const {

	// hsv color with components within the 0-255 range
	HsvColor hsv;

	hsv.s = 255; // hard-coded
	hsv.v = 255; // hard-coded

	static const unsigned char HUE_MAX = 170; 	// blue
	static const unsigned char HUE_MIN = 0;		// red

	double magnitude_norm_complementary = 1 - ((magnitude - min_force_magnitude_)/(max_force_magnitude_ - min_force_magnitude_));

	// map magnitude normalized onto the hue value
	hsv.h = ((HUE_MAX - HUE_MIN) * magnitude_norm_complementary) + HUE_MIN;

	return (hsv);

}

// --------------------------------------------------------------

} /* namespace vis */
} /* namespace sfm */
