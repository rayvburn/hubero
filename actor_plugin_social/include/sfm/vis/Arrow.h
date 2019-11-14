/*
 * Arrow.h
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_SFM_VIS_ARROW_H_
#define INCLUDE_SFM_VIS_ARROW_H_

#include <ignition/math/Angle.hh>
#include <ignition/math/Quaternion.hh>
#include <sfm/vis/MarkerBase.h>

namespace sfm {
namespace vis {

class Arrow : public MarkerBase {

public:

	/// \brief Default constructor
	Arrow();

	/// \brief Sets arrow parameters
	/// \param[in] Max length of an arrow expressed in meters
	/// \param[in] SFM max force is a max allowable force
	/// which SFM algorithm could return; used to scale
	/// arrow's length
	virtual void setParameters(const float &length_meters, const float &sfm_max_force);

	/// \brief Method that creates a simple arrow
	/// of a previously set color
	/// \param[in] Foothold of an arrow
	/// \param[in] Force vector which is used to
	/// determine a marker's orientation and length
	virtual visualization_msgs::Marker create(const ignition::math::Vector3d &pos, const ignition::math::Vector3d &vector) const;

	/// \brief Default destructor
	virtual ~Arrow();

protected:

	/// \brief Max length of an arrow (in meters]
	float max_length_;

	/// \brief Max allowable force which SFM
	/// algorithm could return
	double sfm_max_force_;

};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_SFM_VIS_ARROW_H_ */
