/*
 * Heatmap.h
 *
 *  Created on: Dec 27, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_VIS_HEATMAP_H_
#define INCLUDE_SFM_VIS_HEATMAP_H_

#include "Grid.h"

namespace sfm {
namespace vis {

class Heatmap : public Grid {

public:

	/// \brief Default constructor
	Heatmap();

//	/// \brief
//	/// \param x_start
//	/// \param x_end
//	/// \param y_start
//	/// \param y_end
//	/// \param resolution
//	/// \note Will shadow the base class `createGrid`
//	void createGrid(const float &x_start, const float &x_end, const float &y_start, const float &y_end, const float &resolution);

	/// \brief DEPRECATED
	void scale(const double &max);

	/// \brief Default destructor
	virtual ~Heatmap();

private:

	std::vector<double> magnitudes_v_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_SFM_VIS_HEATMAP_H_ */
