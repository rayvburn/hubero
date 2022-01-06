#pragma once

namespace hubero {

/**
 * @brief Simple wrapper of a time counted in seconds, expressed as double
 */
class Time {
public:
    Time(const double& time = 0.0): time_(time) {}

    double getTime() const {
        return time_;
    }

    static Time computeTimestamp(const Time& begin, const Time& duration) {
        return begin.getTime() + duration.getTime();
    }

    static Time computeDuration(const Time& begin, const Time& finish) {
        return finish.getTime() - begin.getTime();
    }

protected:
    double time_;
};

} // namespace hubero