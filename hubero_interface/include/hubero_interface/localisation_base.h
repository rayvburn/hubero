#pragma once

#include <hubero_common/typedefs.h>
#include <string>

namespace hubero {
namespace interface {

//template<typename T>
class LocalisationBase {
public:
    virtual bool initialize(const std::string& frame = "") {
        frame_id_ = frame;
    }

    virtual void update(const Pose3& pose) {
        pose_ = pose;
    }

    virtual void update(const Pose3& pose, const double& dt) {
        pose_ = pose;
    }

    virtual void update(const Pose3& pose, const Pose3& vel, const Pose3& acc) {
        pose_ = pose;
        vel_ = vel;
        acc_ = acc;
    }

    inline virtual Pose3 getPose() const {
        return pose_;
    }

    inline virtual Pose3 getVel() const {
        return vel_;
    }

    inline virtual Pose3 getAcc() const {
        return acc_;
    }

    virtual ~LocalisationBase() = default;

protected:
    LocalisationBase() = default;

    std::string frame_id_;
    Pose3 pose_;
    Pose3 vel_;
    Pose3 acc_;
};

} // namespace hubero
} // namespace interface