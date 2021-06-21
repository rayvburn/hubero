#include "model_control_base.hpp"

class ModelControlROS: public ModelControlBase {
public:
    ModelControlROS(/* args */);
    virtual void initialize() {};
    virtual void update(const std::string& model_name, const Pose3& pose, const Vector3& vel, const Vector3& acc) = 0;
    virtual void update(const int& model_name, const Pose3& pose, const Vector3& vel, const Vector3& acc) = 0;
    virtual ~ModelControlROS();
};
