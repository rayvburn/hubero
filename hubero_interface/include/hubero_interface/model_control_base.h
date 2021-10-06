#pragma once

#include <hubero_common/typedefs.h>
#include <string>

namespace hubero {
namespace interface {

template<typename Tpose, typename Tvec>
class ModelControlBase {
public:
    // TODO: args list?
    // virtual void initialize() = 0;
    virtual void initialize(
        std::function<void(T)> fun_pose,
        std::function<void(Tvec)> fun_ang_vel,
        std::function<void(Tvec)> fun_lin_vel,
        std::function<void(Tvec)> fun_ang_acc
        std::function<void(Tvec)> fun_lin_acc
    );

    inline virtual void update(
        const Tpose& pose,
        const Tvec& vel_ang,
        const Tvec& vel_lin,
        const Tvec& acc_ang,
        const Tvec& acc_lin
    ) {
        if (fun_pose_ != nullptr) {
            fun_pose_(pose);
        }

        if (fun_ang_vel_ != nullptr) {
            fun_ang_vel_(vel_ang);
        }

        if (fun_lin_vel_ != nullptr) {
            fun_lin_vel_(vel_lin);
        }

        if (fun_ang_acc_ != nullptr) {
            fun_ang_acc_(acc_ang);
        }

        if (fun_lin_acc_ != nullptr) {
            fun_lin_acc_(acc_lin);
        }
    }

    // virtual void update(const int& model_name, const Pose3& pose, const Pose3& vel, const Pose3& acc) = 0;
    virtual ~ModelControlBase() = default;
protected:
    ModelControlBase() = default;

    std::function<void(Tpose)> fun_pose_;
    std::function<void(Tvec)> fun_ang_vel_;
    std::function<void(Tvec)> fun_lin_vel_;
    std::function<void(Tvec)> fun_ang_acc_;
    std::function<void(Tvec)> fun_lin_acc_;
};

} // namespace hubero
} // namespace interface