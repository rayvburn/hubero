/*
 * Point.h
 *
 *  Created on: Apr 16, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_POINT_H_
#define INCLUDE_POINT_H_

#include <MarkerBase.h>







namespace sfm {
namespace vis {

/// \brief Point class extends Base's functionality
/// with a LineList management (may be developed further)
class Point : public MarkerBase {

public:

	/// \brief Default constructor
	Point();

//	/// \brief Sets a color of each line created
//	/// with createLineList() method
//	void setColorLine(const float &r, const float &g, const float &b, const float &alpha);


	/// \brief Default destructor
	virtual ~Point();

private:

//	/// \brief Stores a color a line
//	std_msgs::ColorRGBA color_line_; // FIXME: why there is a separate color instance? there is one already in the base class



};

} /* namespace vis */
} /* namespace sfm */

#endif /* INCLUDE_POINT_H_ */
