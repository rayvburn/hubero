/*
 * Heatmap.h
 *
 *  Created on: Dec 27, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_VIS_HEATMAP_H_
#define INCLUDE_SFM_VIS_HEATMAP_H_

#include "Grid.h"
#include "HeatCell.h"

namespace sfm {
namespace vis {

/// \brief Incorporates Grid and HeatCell classes methods, provides an interface that is easy to use
/// and limits number of objects to manipulate from the main class (Actor) to a single one
class Heatmap : public Grid, public HeatCell {

public:

	/// \brief Default constructor
	Heatmap();

	/// \brief Default destructor
	virtual ~Heatmap();

private:

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_SFM_VIS_HEATMAP_H_ */
