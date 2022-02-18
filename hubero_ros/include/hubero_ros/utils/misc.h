#pragma once

#include <boost/array.hpp>

namespace hubero {

/**
 * @brief Sets ideal covariance in the @ref cov array
 * @details boost array used here to comply with ROS messages
 */
void setIdealCovariance(boost::array<double, 36>& cov);

} // namespace hubero
