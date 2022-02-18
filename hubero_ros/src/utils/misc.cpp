#include <hubero_ros/utils/misc.h>

namespace hubero {

void setIdealCovariance(boost::array<double, 36>& cov) {
	// update covariance - note that pose is perfectly known
	std::fill(cov.begin(), cov.end(), 0);
	// ones on diagonal
	for (unsigned int i = 0; i < cov.size(); i += 1 + sqrt(cov.size())) {
		cov[i] = 1e-06;
	}
}

} // namespace hubero
