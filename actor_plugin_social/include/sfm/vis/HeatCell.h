/*
 * Cell.h
 *
 *  Created on: Dec 27, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_VIS_HEATCELL_H_
#define INCLUDE_SFM_VIS_HEATCELL_H_

#include "MarkerBase.h"
#include <actor/ros_interface/ParamLoader.h>
#include <vector>

namespace sfm {
namespace vis {

class HeatCell : public MarkerBase {

public:

	// TODO: Grid is a derivate class from the Arrow
	HeatCell();

	virtual void setParameters(const double &min_force_magnitude, const double &max_force_magnitude, const double &resolution);

	virtual visualization_msgs::Marker create(const ignition::math::Vector3d &pos, const double &force_magnitude) const;

	virtual ~HeatCell();

private:

	double max_force_magnitude_;
	double min_force_magnitude_;
	double resolution_;

	/// \section Reference: https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
	typedef struct {
	    unsigned char r;
	    unsigned char g;
	    unsigned char b;
	} RgbColor;

	typedef struct {
	    unsigned char h;
	    unsigned char s;
	    unsigned char v;
	} HsvColor;

	RgbColor HsvToRgb(const HsvColor &hsv) const;
	HsvColor RgbToHsv(const RgbColor &rgb) const;

	HsvColor convertMagnitudeToHSV(const double &magnitude) const;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_SFM_VIS_HEATCELL_H_ */
