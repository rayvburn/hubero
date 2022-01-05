#pragma once

#include <hubero_common/time.h>
#include <gazebo/common/Time.hh>

namespace hubero {

class TimeGazebo: Time {
public:
    TimeGazebo(const gazebo::common::Time& time);
};

} // namespace hubero