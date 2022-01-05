#include <hubero_gazebo/time_gazebo.h>

namespace hubero {

TimeGazebo::TimeGazebo(const gazebo::common::Time& time): Time(time.Double()) {}

} // namespace hubero
