#pragma once

#include <hubero_common/typedefs.h>
#include <string>

namespace hubero {
namespace interface {

class ModelControlBase {
public:
    // TODO: args list?
    virtual void initialize() = 0;
    virtual void update(const std::string& model_name, const Pose3& pose, const Vector3& vel, const Vector3& acc) = 0;
    virtual void update(const int& model_name, const Pose3& pose, const Vector3& vel, const Vector3& acc) = 0;
    virtual ~ModelControlBase() = default;
protected:
    ModelControlBase() = default;
};

} // namespace hubero
} // namespace interface